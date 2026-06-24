#!/usr/bin/env python3

import html
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

import rosgraph
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState
from rosgraph_msgs.msg import Clock


DEFAULT_CMD_LINEAR_X = 1.0
DEFAULT_CMD_YAW_RATE = 1.0
CMD_VEL_RATE_HZ = 20.0


class ManagedProcess:
    def __init__(self, key, command, log_file):
        self.key = key
        self.command = command
        self.log_file = log_file
        self.started_at = time.time()
        self.process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    def running(self):
        return self.process.poll() is None

    def pid(self):
        return self.process.pid

    def returncode(self):
        return self.process.poll()


class GazeboSessionManager:
    def __init__(self):
        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 8088))
        self.world_name = rospy.get_param(
            "~world_name",
            "$(find gazebo_sim_worlds)/worlds/flat_empty.world",
        )
        self.default_z = float(rospy.get_param("~default_z", 0.181))
        self.auto_spacing_x = float(rospy.get_param("~auto_spacing_x", 1.6))
        self.auto_spacing_y = float(rospy.get_param("~auto_spacing_y", 1.2))
        self.auto_min_distance = float(rospy.get_param("~auto_min_distance", 1.0))
        self.cleanup_on_shutdown = bool(rospy.get_param("~cleanup_on_shutdown", True))
        self.log_dir = os.path.expanduser(
            rospy.get_param("~log_dir", "~/.ros/gazebo_session_manager")
        )
        os.makedirs(self.log_dir, exist_ok=True)
        self.processes = {}
        self.robot_initial_poses = {}
        self.robot_command_defaults = {}
        self.last_message = "ready"
        self.lock = threading.RLock()
        self.event_condition = threading.Condition()
        self.event_seq = 0
        self.master_cache = None
        self.master_cache_time = 0.0
        self.last_clock_wall_time = 0.0
        self.last_model_states = None
        self.last_model_states_wall_time = 0.0
        self._cleanup_started = False
        self.clock_sub = rospy.Subscriber("/clock", Clock, self._clock_callback, queue_size=1)
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1
        )

    def start_process(self, key, command):
        with self.lock:
            proc = self.processes.get(key)
            if proc and proc.running():
                return "%s is already running, pid=%d" % (key, proc.pid())

            log_path = os.path.join(self.log_dir, "%s.log" % key.replace(":", "_"))
            log_file = open(log_path, "ab", buffering=0)
            proc = ManagedProcess(key, command, log_file)
            self.processes[key] = proc
            self.notify_update()
            return "started %s, pid=%d, log=%s" % (key, proc.pid(), log_path)

    def terminate_process(self, key, timeout=5.0):
        with self.lock:
            proc = self.processes.get(key)
            if not proc or not proc.running():
                return "%s is not running" % key
            return self._terminate_managed_process(proc, timeout)

    def _terminate_managed_process(self, proc, timeout=5.0):
        pid = proc.pid()
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            return "%s already exited" % proc.key

        for sig, name in [
            (signal.SIGINT, "SIGINT"),
            (signal.SIGTERM, "SIGTERM"),
            (signal.SIGKILL, "SIGKILL"),
        ]:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return "%s exited" % proc.key

            deadline = time.time() + timeout
            while time.time() < deadline:
                if not proc.running():
                    self.notify_update()
                    return "stopped %s with %s" % (proc.key, name)
                time.sleep(0.1)
        self.notify_update()
        return "sent SIGKILL to %s" % proc.key

    def start_gzserver(self):
        proc = self.processes.get("gzserver")
        if proc and proc.running():
            status = self.gazebo_status()
            if status["ready"]:
                return "gzserver is already running and ready, pid=%d" % proc.pid()
            stale_msg = self.terminate_process("gzserver", timeout=2.0)
            self._kill_gazebo_runtime()
            return "%s; stale Gazebo runtime was not ready, restarted: %s" % (
                stale_msg,
                self._start_gzserver_and_wait(),
            )
        self._kill_gazebo_runtime()
        return self._start_gzserver_and_wait()

    def _start_gzserver_and_wait(self):
        start_msg = self.start_process(
            "gzserver",
            [
                "roslaunch",
                "gazebo_ros",
                "empty_world.launch",
                "world_name:=%s" % self.world_name,
                "use_sim_time:=true",
                "gui:=false",
                "headless:=false",
                "debug:=false",
            ],
        )
        status = self._wait_for_gazebo_ready(timeout=25.0)
        if status["ready"]:
            return "%s; Gazebo ready" % start_msg
        return "%s; Gazebo process started but not ready: %s" % (
            start_msg,
            status["summary"],
        )

    def start_gzclient(self):
        status = self.gazebo_status()
        if not status["ready"]:
            raise RuntimeError(
                "Gazebo server is not ready, cannot start GUI: %s" % status["summary"]
            )
        return self.start_process("gzclient", ["gzclient"])

    def stop_gzclient(self):
        msg = self.terminate_process("gzclient", timeout=2.0)
        self._run_quiet(["rosnode", "kill", "/gazebo_gui"])
        return msg

    def start_rviz(self):
        robot_names = sorted(self._known_robot_names(timeout=0.2), key=self._robot_sort_key)
        if not robot_names:
            raise RuntimeError("no UGV models are available for RViz")
        self.terminate_process("rviz", timeout=1.0)
        self._run_quiet(["rosnode", "kill", "/rviz"])
        config_path = self._write_rviz_config(robot_names)
        return self.start_process("rviz", ["rviz", "-d", config_path])

    def stop_rviz(self):
        msg = self.terminate_process("rviz", timeout=2.0)
        self._run_quiet(["rosnode", "kill", "/rviz"])
        return msg

    def start_vrpn_client(self):
        robot_names = sorted(self._known_robot_names(timeout=0.2), key=self._robot_sort_key)
        if not robot_names:
            raise RuntimeError("no UGV models are available for VRPN client")
        self.terminate_process("vrpn_client", timeout=2.0)
        self._run_quiet(["rosnode", "kill", "/vrpn_client_node"])
        return self.start_process(
            "vrpn_client",
            [
                "roslaunch",
                "vrpn_client_ros",
                "sample.launch",
                "server:=127.0.0.1",
            ],
        )

    def start_robot(self, name, x, y, z, yaw):
        with self.lock:
            self._validate_name(name)
            values = [x, y, z, yaw]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("pose contains non-finite values")
            self._require_gazebo_ready()
            if name in self._known_robot_names(timeout=0.2):
                raise RuntimeError("UGV '%s' already exists or is being spawned" % name)
            self.robot_initial_poses[name] = {
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }
            self.robot_command_defaults.setdefault(
                name,
                {"linear_x": DEFAULT_CMD_LINEAR_X, "yaw_rate": DEFAULT_CMD_YAW_RATE},
            )
            return self.start_process(
                "robot:%s" % name,
                [
                    "roslaunch",
                    "gazebo_sim_scout",
                    "spawn_accurate.launch",
                    "ns:=%s" % name,
                    "model_name:=%s" % name,
                    "x:=%.6f" % x,
                    "y:=%.6f" % y,
                    "z:=%.6f" % z,
                    "yaw:=%.6f" % yaw,
                    "robot_description_param:=/%s/robot_description" % name,
                    "odom_topic:=/%s/odom" % name,
                    "odom_frame:=odom",
                    "base_frame:=%s/base_link" % name,
                    "tf_prefix:=%s" % name,
                    "frame_prefix:=%s/" % name,
                    "sensor_ns:=%s" % name,
                    "robot_state_publisher_ns:=%s" % name,
                ],
            )

    def start_next_robot(self):
        with self.lock:
            spec = self.next_robot_spec()
            message = self.start_robot(
                spec["name"], spec["x"], spec["y"], spec["z"], spec["yaw"]
            )
        return "%s at x=%.3f, y=%.3f, yaw=%.3f" % (
            message,
            spec["x"],
            spec["y"],
            spec["yaw"],
        )

    def delete_robot(self, name):
        self._validate_name(name)
        messages = []
        messages.append(self.stop_robot_command(name))
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
            delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            resp = delete_model(name)
            messages.append("delete_model %s: %s" % (name, resp.status_message))
        except Exception as exc:
            messages.append("delete_model %s failed: %s" % (name, exc))
        messages.append(self.terminate_process("robot:%s" % name, timeout=3.0))
        self.robot_initial_poses.pop(name, None)
        self.robot_command_defaults.pop(name, None)
        return "; ".join(messages)

    def delete_last_robot(self):
        name = self.last_robot_name()
        if not name:
            raise RuntimeError("no UGV model is available to delete")
        return "delete last %s: %s" % (name, self.delete_robot(name))

    def reset_robot_pose(self, name, x, y, z, yaw):
        self._validate_name(name)
        if not all(math.isfinite(v) for v in [x, y, z, yaw]):
            raise ValueError("pose contains non-finite values")

        rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(yaw * 0.5)
        state.pose.orientation.w = math.cos(yaw * 0.5)
        resp = set_state(state)
        return "set_model_state %s: %s" % (name, resp.status_message)

    def reset_robot_to_initial_pose(self, name):
        self._validate_name(name)
        pose = self._initial_pose_for(name)
        return self.reset_robot_pose(name, pose["x"], pose["y"], pose["z"], pose["yaw"])

    def start_robot_command(self, name, linear_x, yaw_rate):
        self._validate_name(name)
        if not all(math.isfinite(v) for v in [linear_x, yaw_rate]):
            raise ValueError("velocity command contains non-finite values")
        self.robot_command_defaults[name] = {
            "linear_x": linear_x,
            "yaw_rate": yaw_rate,
        }
        message = "linear:\n  x: %.6f\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: %.6f\n" % (
            linear_x,
            yaw_rate,
        )
        return self.start_process(
            "cmd_vel:%s" % name,
            [
                "rostopic",
                "pub",
                "-r",
                "%.1f" % CMD_VEL_RATE_HZ,
                "/%s/cmd_vel" % name,
                "geometry_msgs/Twist",
                message,
            ],
        )

    def stop_robot_command(self, name):
        self._validate_name(name)
        stop_msg = self.terminate_process("cmd_vel:%s" % name, timeout=1.0)
        zero_msg = "linear:\n  x: 0.0\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n"
        self._run_quiet(
            [
                "rostopic",
                "pub",
                "-1",
                "/%s/cmd_vel" % name,
                "geometry_msgs/Twist",
                zero_msg,
            ]
        )
        return "%s; sent zero cmd_vel to %s" % (stop_msg, name)

    def kill_session(self):
        return self.cleanup_session("manual")

    def cleanup_session(self, reason):
        messages = []
        with self.lock:
            if self._cleanup_started:
                return "cleanup already running"
            self._cleanup_started = True

        try:
            messages.append("cleanup reason: %s" % reason)
            with self.lock:
                keys = list(self.processes.keys())
            for key in keys:
                messages.append(self.terminate_process(key, timeout=2.0))

            node_names = self._rosnode_list()
            kill_targets = [
                n
                for n in node_names
                if n in ["/gazebo", "/gazebo_gui", "/gazebo_vrpn_server", "/vrpn_client_node", "/rviz"]
                or "scout_skid_steer_controller" in n
                or "gazebo_model_odom" in n
                or "robot_state_publisher" in n
                or "spawn_scout_model" in n
                or "controller_spawner" in n
            ]
            if kill_targets:
                self._run_quiet(["rosnode", "kill"] + kill_targets)
                messages.append("rosnode kill: " + ", ".join(kill_targets))

            self._kill_gazebo_runtime()
            messages.append("killed stale gzserver/gzclient/rviz processes")
            with self.lock:
                self.processes.clear()
                self.robot_initial_poses.clear()
                self.robot_command_defaults.clear()
            return "; ".join(messages)
        finally:
            with self.lock:
                self._cleanup_started = False

    def process_status(self):
        with self.lock:
            rows = []
            for key in sorted(self.processes):
                proc = self.processes[key]
                rows.append(
                    {
                        "key": key,
                        "pid": proc.pid(),
                        "running": proc.running(),
                        "returncode": proc.returncode(),
                        "age": time.time() - proc.started_at,
                    }
                )
            return rows

    def notify_update(self):
        with self.event_condition:
            self.event_seq += 1
            self.event_condition.notify_all()

    def wait_for_update(self, last_seq, timeout=1.0):
        with self.event_condition:
            if self.event_seq <= last_seq:
                self.event_condition.wait(timeout=timeout)
            return self.event_seq

    def snapshot_payload(self):
        gazebo_status = self.gazebo_status()
        next_robot = self.next_robot_spec()
        last_robot_name = self.last_robot_name()
        robot_rows = self.robot_rows()
        return {
            "seq": self.event_seq,
            "last_message": self.last_message,
            "gazebo_ready": gazebo_status["ready"],
            "gazebo_readiness_text": "ready" if gazebo_status["ready"] else "not ready",
            "gazebo_readiness_summary": gazebo_status["summary"],
            "next_robot_text": "Next: %s at (%.3f, %.3f, %.3f)" % (
                next_robot["name"],
                next_robot["x"],
                next_robot["y"],
                next_robot["z"],
            ),
            "delete_last_text": "Delete Last UGV (%s)" % (
                last_robot_name if last_robot_name else "none"
            ),
            "robot_forms_html": "\n".join(
                '<form id="robot_{name}" method="post"></form>'.format(
                    name=html.escape(r["name"])
                )
                for r in robot_rows
            ),
            "robot_rows_html": self.robot_rows_html(robot_rows),
            "process_rows_html": self.process_rows_html(),
        }

    def process_rows_html(self):
        rows = self.process_status()
        rows_html = "\n".join(
            "<tr><td>{key}</td><td>{pid}</td><td>{running}</td><td>{returncode}</td><td>{age:.1f}s</td></tr>".format(
                key=html.escape(r["key"]),
                pid=r["pid"],
                running="running" if r["running"] else "stopped",
                returncode="" if r["returncode"] is None else r["returncode"],
                age=r["age"],
            )
            for r in rows
        )
        if not rows_html:
            rows_html = "<tr><td colspan='5'>No managed processes yet.</td></tr>"
        return rows_html

    def robot_rows_html(self, robot_rows=None):
        if robot_rows is None:
            robot_rows = self.robot_rows()
        rows_html = "\n".join(self.render_robot_row(r) for r in robot_rows)
        if not rows_html:
            rows_html = "<tr><td colspan='10'>No UGV models detected.</td></tr>"
        return rows_html

    def render_robot_row(self, row):
        name = row["name"]
        form_id = "robot_%s" % name
        initial = row["initial"]
        command = row["command"]
        status = []
        status.append("gazebo" if row["in_gazebo"] else "not in gazebo")
        status.append("cmd running" if row["cmd_running"] else "cmd stopped")
        return """
        <tr>
          <td>
            {name}
            <input form="{form_id}" type="hidden" name="name" value="{name}">
          </td>
          <td>{status}</td>
          <td><input form="{form_id}" class="compact" name="init_x" value="{init_x:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_y" value="{init_y:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_z" value="{init_z:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_yaw" value="{init_yaw:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="cmd_x" value="{cmd_x:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="cmd_yaw" value="{cmd_yaw:.3f}"></td>
          <td>
            <button form="{form_id}" name="action" value="start_cmd">Start</button>
            <button form="{form_id}" class="stop" name="action" value="stop_cmd">Stop</button>
          </td>
          <td>
            <button form="{form_id}" name="action" value="reset_robot_initial">Reset Pose</button>
          </td>
        </tr>""".format(
            form_id=html.escape(form_id),
            name=html.escape(name),
            status=html.escape(", ".join(status)),
            init_x=initial["x"],
            init_y=initial["y"],
            init_z=initial["z"],
            init_yaw=initial["yaw"],
            cmd_x=command["linear_x"],
            cmd_yaw=command["yaw_rate"],
        )

    def robot_rows(self):
        current_states = self._known_robot_states(timeout=0.05)
        names = set(current_states)
        names.update(
            key.split(":", 1)[1]
            for key, proc in self.processes.items()
            if key.startswith("robot:") and proc.running()
        )
        names.update(self.robot_initial_poses)

        rows = []
        for name in sorted(names, key=self._robot_sort_key):
            if name not in self.robot_initial_poses and name in current_states:
                self.robot_initial_poses[name] = dict(current_states[name])
            initial_pose = self._initial_pose_for(name, current_states)
            command = self.robot_command_defaults.setdefault(
                name,
                {"linear_x": DEFAULT_CMD_LINEAR_X, "yaw_rate": DEFAULT_CMD_YAW_RATE},
            )
            rows.append(
                {
                    "name": name,
                    "initial": initial_pose,
                    "command": command,
                    "cmd_running": self._process_running("cmd_vel:%s" % name),
                    "robot_running": self._process_running("robot:%s" % name),
                    "in_gazebo": name in current_states,
                }
            )
        return rows

    def next_robot_spec(self):
        existing_names = self._known_robot_names(timeout=0.05)
        index = 1
        while "ugv%d" % index in existing_names:
            index += 1

        occupied = self._known_robot_positions(timeout=0.05)
        x, y = self._next_free_position(occupied)
        return {
            "name": "ugv%d" % index,
            "x": x,
            "y": y,
            "z": self.default_z,
            "yaw": 0.0,
        }

    def last_robot_name(self):
        best_name = ""
        best_index = -1
        for name in self._known_robot_names(timeout=0.05):
            match = re.fullmatch(r"ugv(\d+)", name)
            if not match:
                continue
            index = int(match.group(1))
            if index > best_index:
                best_name = name
                best_index = index
        return best_name

    def _initial_pose_for(self, name, current_states=None):
        pose = self.robot_initial_poses.get(name)
        if pose:
            return pose
        if current_states is None:
            current_states = self._known_robot_states(timeout=0.05)
        pose = current_states.get(name)
        if pose:
            self.robot_initial_poses[name] = dict(pose)
            return pose
        return {"x": 0.0, "y": 0.0, "z": self.default_z, "yaw": 0.0}

    def _process_running(self, key):
        proc = self.processes.get(key)
        return bool(proc and proc.running())

    @staticmethod
    def _robot_sort_key(name):
        match = re.fullmatch(r"ugv(\d+)", name)
        if match:
            return int(match.group(1)), name
        return 1000000, name

    def gazebo_status(self):
        nodes, services = self._ros_master_state()
        has_gazebo_node = "/gazebo" in nodes
        has_vrpn_node = "/gazebo_vrpn_server" in nodes
        has_spawn_service = "/gazebo/spawn_urdf_model" in services
        has_clock = (time.time() - self.last_clock_wall_time) < 1.0
        ready = has_gazebo_node and has_spawn_service and has_clock
        parts = [
            "/gazebo node=%s" % self._yes_no(has_gazebo_node),
            "spawn service=%s" % self._yes_no(has_spawn_service),
            "/clock=%s" % self._yes_no(has_clock),
            "VRPN node=%s" % self._yes_no(has_vrpn_node),
        ]
        return {
            "ready": ready,
            "gazebo_node": has_gazebo_node,
            "spawn_service": has_spawn_service,
            "clock": has_clock,
            "vrpn_node": has_vrpn_node,
            "summary": ", ".join(parts),
        }

    def _wait_for_gazebo_ready(self, timeout):
        deadline = time.time() + timeout
        status = self.gazebo_status()
        while time.time() < deadline:
            if status["ready"]:
                return status
            time.sleep(0.5)
            status = self.gazebo_status()
        return status

    def _yes_no(self, value):
        return "yes" if value else "no"

    def _validate_name(self, name):
        if not name:
            raise ValueError("robot name is empty")
        allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")
        if any(ch not in allowed for ch in name):
            raise ValueError("robot name may only contain letters, numbers, and '_'")

    def _run_quiet(self, command):
        try:
            return subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                text=True,
                timeout=5.0,
            )
        except Exception:
            return None

    def _kill_gazebo_runtime(self):
        node_names = self._rosnode_list()
        kill_targets = [
            n
            for n in node_names
            if n in ["/gazebo", "/gazebo_gui", "/gazebo_vrpn_server", "/vrpn_client_node", "/rviz"]
        ]
        if kill_targets:
            self._run_quiet(["rosnode", "kill"] + kill_targets)
            time.sleep(0.5)

        for proc_name in ["gzserver", "gzclient", "rviz"]:
            self._run_quiet(["pkill", "-TERM", "-x", proc_name])
        time.sleep(1.0)
        for proc_name in ["gzserver", "gzclient", "rviz"]:
            self._run_quiet(["pkill", "-KILL", "-x", proc_name])

    def _require_gazebo_ready(self):
        if not self._service_available("/gazebo/spawn_urdf_model", timeout=0.5):
            raise RuntimeError(
                "Gazebo is not ready: /gazebo/spawn_urdf_model is unavailable. "
                "Click 'Start gzserver' first and wait until it is running."
            )
        if not self._clock_available(timeout=1.0):
            raise RuntimeError(
                "Gazebo is not publishing /clock. Use 'Kill Gazebo Session', then "
                "start gzserver again before adding a UGV."
            )

    def _service_available(self, service_name, timeout):
        del timeout
        _, services = self._ros_master_state()
        return service_name in services

    def _clock_available(self, timeout):
        return (time.time() - self.last_clock_wall_time) < timeout

    def _rosnode_list(self):
        nodes, _ = self._ros_master_state()
        return sorted(nodes)

    def _ros_master_state(self):
        now = time.time()
        with self.lock:
            if self.master_cache and now - self.master_cache_time < 1.0:
                return self.master_cache
        nodes = set()
        services = set()
        try:
            publishers, subscribers, service_entries = rosgraph.Master(
                "/gazebo_session_webui"
            ).getSystemState()
            for topic, publishers in publishers:
                del topic
                nodes.update(publishers)
            for topic, subscribers in subscribers:
                del topic
                nodes.update(subscribers)
            for service, providers in service_entries:
                services.add(service)
                nodes.update(providers)
        except Exception:
            pass
        with self.lock:
            self.master_cache = (nodes, services)
            self.master_cache_time = now
        return nodes, services

    def _model_states(self, timeout):
        del timeout
        if time.time() - self.last_model_states_wall_time > 2.0:
            return None
        return self.last_model_states

    def _gazebo_model_names(self, timeout):
        states = self._model_states(timeout)
        if not states:
            return set()
        return set(states.name)

    def _known_robot_names(self, timeout):
        names = {
            key.split(":", 1)[1]
            for key, proc in self.processes.items()
            if key.startswith("robot:") and proc.running()
        }
        names.update(self.robot_initial_poses)
        for name in self._gazebo_model_names(timeout):
            if name.startswith("ugv"):
                names.add(name)
        return names

    def _known_robot_positions(self, timeout):
        positions = [
            (pose["x"], pose["y"]) for pose in self.robot_initial_poses.values()
        ]
        states = self._model_states(timeout)
        if not states:
            return positions
        for name, pose in zip(states.name, states.pose):
            if name.startswith("ugv") and name not in self.robot_initial_poses:
                positions.append((pose.position.x, pose.position.y))
        return positions

    def _known_robot_states(self, timeout):
        states = self._model_states(timeout)
        if not states:
            return {}
        result = {}
        for name, pose in zip(states.name, states.pose):
            if name.startswith("ugv"):
                result[name] = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "yaw": self._yaw_from_pose(pose),
                }
        return result

    def _clock_callback(self, _msg):
        was_stale = (time.time() - self.last_clock_wall_time) > 1.0
        self.last_clock_wall_time = time.time()
        if was_stale:
            self.notify_update()

    def _model_states_callback(self, msg):
        old_names = set(self._known_robot_states(timeout=0.0))
        self.last_model_states = msg
        self.last_model_states_wall_time = time.time()
        new_names = set(self._known_robot_states(timeout=0.0))
        if old_names != new_names:
            self.notify_update()

    def _write_rviz_config(self, robot_names):
        path = os.path.join(self.log_dir, "dynamic_ugv.rviz")
        with open(path, "w", encoding="utf-8") as config:
            config.write(self._rviz_config_text(robot_names))
        return path

    def _rviz_config_text(self, robot_names):
        expanded = "\n".join(
            "\n".join(
                (
                    "        - /{name} RobotModel1".format(name=name),
                    "        - /{name} LaserScan1".format(name=name),
                    "        - /{name} Image1".format(name=name),
                )
            )
            for name in robot_names
        )
        robot_displays = "\n".join(
            self._rviz_robot_displays(name) for name in robot_names
        )
        return """Panels:
  - Class: rviz/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
{expanded}
      Splitter Ratio: 0.55
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: false
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
{robot_displays}
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: false
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Theta std deviation: 0.2617993950843811
      Topic: /initialpose
      X std deviation: 0.5
      Y std deviation: 0.5
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.55
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.27
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: false
  Views:
    collapsed: false
  Width: 1280
  X: 100
  Y: 80
""".format(expanded=expanded, robot_displays=robot_displays)

    @staticmethod
    def _rviz_robot_displays(name):
        return "\n".join(
            (
                GazeboSessionManager._rviz_robot_model_display(name),
                GazeboSessionManager._rviz_laserscan_display(name),
                GazeboSessionManager._rviz_image_display(name),
            )
        )

    @staticmethod
    def _rviz_robot_model_display(name):
        return """    - Alpha: 1
      Class: rviz/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: {name} RobotModel
      Robot Description: /{name}/robot_description
      TF Prefix: {name}
      Update Interval: 0
      Value: true
      Visual Enabled: true""".format(name=name)

    @staticmethod
    def _rviz_laserscan_display(name):
        return """    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 8
        Min Value: 0
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/LaserScan
      Color: 255; 255; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: {name} LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.03
      Style: Points
      Topic: /{name}/scan
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true""".format(name=name)

    @staticmethod
    def _rviz_image_display(name):
        return """    - Class: rviz/Image
      Enabled: true
      Image Topic: /{name}/camera/color/image_raw
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: {name} Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true""".format(name=name)

    @staticmethod
    def _yaw_from_pose(pose):
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _next_free_position(self, occupied):
        for ix, iy in self._grid_offsets(limit=12):
            x = ix * self.auto_spacing_x
            y = iy * self.auto_spacing_y
            if self._position_is_free(x, y, occupied):
                return x, y
        raise RuntimeError("could not find a free UGV spawn position")

    def _grid_offsets(self, limit):
        yield 0, 0
        for radius in range(1, limit + 1):
            yield 0, radius
            yield 0, -radius
            yield radius, 0
            yield -radius, 0
            for ix in range(1, radius + 1):
                yield ix, radius
                yield -ix, radius
                yield ix, -radius
                yield -ix, -radius
            for iy in range(1, radius):
                yield radius, iy
                yield radius, -iy
                yield -radius, iy
                yield -radius, -iy

    def _position_is_free(self, x, y, occupied):
        min_distance_sq = self.auto_min_distance * self.auto_min_distance
        for ox, oy in occupied:
            dx = x - ox
            dy = y - oy
            if dx * dx + dy * dy < min_distance_sq:
                return False
        return True


class RequestHandler(BaseHTTPRequestHandler):
    manager = None

    def do_GET(self):
        if self.path == "/events":
            self._send_events()
            return
        try:
            self._send_html(self._render_page())
        except (BrokenPipeError, ConnectionResetError):
            rospy.logdebug("web client disconnected during GET response")

    def _send_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        seq = -1
        try:
            while not rospy.is_shutdown():
                seq = self.manager.wait_for_update(seq, timeout=1.0)
                payload = json.dumps(self.manager.snapshot_payload())
                self.wfile.write(("data: %s\n\n" % payload).encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            rospy.logdebug("web client disconnected from event stream")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8")
        form = parse_qs(raw)

        try:
            message = self._handle_action(form)
            self.manager.last_message = message
        except Exception as exc:
            rospy.logwarn("web action failed: %s", exc)
            self.manager.last_message = "ERROR: %s" % exc
        self.manager.notify_update()

        if self.path == "/api/action":
            self._send_json({"ok": True, "message": self.manager.last_message})
            return

        try:
            self._send_html(self._render_page())
        except (BrokenPipeError, ConnectionResetError):
            rospy.logdebug("web client disconnected during POST response")

    def log_message(self, fmt, *args):
        rospy.loginfo("webui %s - %s", self.address_string(), fmt % args)

    def _handle_action(self, form):
        action = self._get(form, "action", "")
        name = self._get(form, "name", "ugv1").strip()
        x = self._float(form, "x", 0.0)
        y = self._float(form, "y", 0.0)
        z = self._float(form, "z", self.manager.default_z)
        yaw = self._float(form, "yaw", 0.0)

        if action == "start_gzserver":
            return self.manager.start_gzserver()
        if action == "start_gzclient":
            return self.manager.start_gzclient()
        if action == "stop_gzclient":
            return self.manager.stop_gzclient()
        if action == "start_rviz":
            return self.manager.start_rviz()
        if action == "stop_rviz":
            return self.manager.stop_rviz()
        if action == "start_vrpn_client":
            return self.manager.start_vrpn_client()
        if action == "start_next_robot":
            return self.manager.start_next_robot()
        if action == "start_robot":
            return self.manager.start_robot(name, x, y, z, yaw)
        if action == "delete_robot":
            return self.manager.delete_last_robot()
        if action == "reset_robot_initial":
            reset_pose = {
                "x": self._float(form, "init_x", 0.0),
                "y": self._float(form, "init_y", 0.0),
                "z": self._float(form, "init_z", self.manager.default_z),
                "yaw": self._float(form, "init_yaw", 0.0),
            }
            self.manager.robot_initial_poses[name] = reset_pose
            return self.manager.reset_robot_pose(
                name,
                reset_pose["x"],
                reset_pose["y"],
                reset_pose["z"],
                reset_pose["yaw"],
            )
        if action == "start_cmd":
            return self.manager.start_robot_command(
                name,
                self._float(form, "cmd_x", DEFAULT_CMD_LINEAR_X),
                self._float(form, "cmd_yaw", DEFAULT_CMD_YAW_RATE),
            )
        if action == "stop_cmd":
            return self.manager.stop_robot_command(name)
        if action == "kill_session":
            return self.manager.kill_session()
        return "unknown action: %s" % action

    def _render_page(self):
        message = html.escape(self.manager.last_message)
        gazebo_status = self.manager.gazebo_status()
        readiness_class = "ok" if gazebo_status["ready"] else "bad"
        readiness_text = "ready" if gazebo_status["ready"] else "not ready"
        readiness_summary = html.escape(gazebo_status["summary"])
        next_robot = self.manager.next_robot_spec()
        last_robot_name = self.manager.last_robot_name()
        last_robot_text = last_robot_name if last_robot_name else "none"
        robot_rows = self.manager.robot_rows()
        robot_forms_html = "\n".join(
            '<form id="robot_{name}" method="post"></form>'.format(
                name=html.escape(r["name"])
            )
            for r in robot_rows
        )
        robot_rows_html = "\n".join(self._render_robot_row(r) for r in robot_rows)
        if not robot_rows_html:
            robot_rows_html = "<tr><td colspan='10'>No UGV models detected.</td></tr>"
        rows = self.manager.process_status()
        rows_html = "\n".join(
            "<tr><td>{key}</td><td>{pid}</td><td>{running}</td><td>{returncode}</td><td>{age:.1f}s</td></tr>".format(
                key=html.escape(r["key"]),
                pid=r["pid"],
                running="running" if r["running"] else "stopped",
                returncode="" if r["returncode"] is None else r["returncode"],
                age=r["age"],
            )
            for r in rows
        )
        if not rows_html:
            rows_html = "<tr><td colspan='5'>No managed processes yet.</td></tr>"

        return """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Gazebo Session Manager</title>
  <style>
    body {{ font-family: sans-serif; margin: 24px; background: #f5f6f8; color: #1f2933; }}
    h1 {{ margin: 0 0 16px; }}
    .bar {{ padding: 10px 12px; background: #ffffff; border: 1px solid #d8dde6; margin-bottom: 16px; }}
    .status {{ padding: 10px 12px; background: #ffffff; border: 1px solid #d8dde6; margin-bottom: 16px; }}
    .ok {{ color: #166534; font-weight: 700; }}
    .bad {{ color: #991b1b; font-weight: 700; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; align-items: start; }}
    section {{ background: #ffffff; border: 1px solid #d8dde6; padding: 14px; }}
    h2 {{ margin: 0 0 12px; font-size: 18px; }}
    label {{ display: inline-block; margin: 4px 8px 4px 0; }}
    input {{ width: 92px; padding: 5px; }}
    input[name=name] {{ width: 120px; }}
    input.compact {{ width: 72px; }}
    .hint {{ margin: 0 0 8px; color: #475569; }}
    button {{ margin: 4px 6px 4px 0; padding: 7px 11px; cursor: pointer; }}
    button.danger {{ background: #b91c1c; color: white; border: 1px solid #7f1d1d; }}
    button.stop {{ background: #334155; color: white; border: 1px solid #1e293b; }}
    .wide {{ grid-column: 1 / -1; }}
    table {{ border-collapse: collapse; width: 100%; background: #ffffff; }}
    th, td {{ border: 1px solid #d8dde6; padding: 7px; text-align: left; }}
    th {{ background: #edf1f5; }}
  </style>
</head>
<body>
  <h1>Gazebo Session Manager</h1>
  <div class="bar"><strong>Last action:</strong> <span id="last-message">{message}</span></div>
  <div class="status">
    <strong>Gazebo readiness:</strong>
    <span id="readiness-text" class="{readiness_class}">{readiness_text}</span>
    <span id="readiness-summary">({readiness_summary})</span>
  </div>

  <div class="grid">
    <section>
      <h2>Gazebo</h2>
      <form method="post">
        <button name="action" value="start_gzserver">Start gzserver</button>
        <button name="action" value="start_gzclient">Start Gazebo GUI</button>
        <button name="action" value="stop_gzclient">Stop Gazebo GUI</button>
        <button name="action" value="start_rviz">Start RViz</button>
        <button name="action" value="stop_rviz">Stop RViz</button>
        <button name="action" value="start_vrpn_client">Start VRPN Client</button>
        <button class="danger" name="action" value="kill_session">Kill Gazebo Session</button>
      </form>
    </section>

    <section>
      <h2>UGV</h2>
      <form method="post">
        <p id="next-robot" class="hint">Next: {next_name} at ({next_x:.3f}, {next_y:.3f}, {next_z:.3f})</p>
        <button name="action" value="start_next_robot">Add UGV</button>
        <button id="delete-last-button" class="danger" name="action" value="delete_robot">Delete Last UGV ({last_robot})</button>
      </form>
    </section>

    <section class="wide">
      <h2>UGV List</h2>
      <div id="robot-forms">{robot_forms}</div>
      <table>
        <tr>
          <th>name</th>
          <th>state</th>
          <th>init x</th>
          <th>init y</th>
          <th>init z</th>
          <th>init yaw</th>
          <th>cmd x</th>
          <th>cmd yaw</th>
          <th>command</th>
          <th>pose</th>
        </tr>
        <tbody id="robot-rows">{robot_rows}</tbody>
      </table>
    </section>
  </div>

  <h2>Managed Processes</h2>
  <table>
    <tr><th>key</th><th>pid</th><th>state</th><th>return code</th><th>age</th></tr>
    <tbody id="process-rows">{rows}</tbody>
  </table>
  <script>
    document.addEventListener('submit', async (event) => {{
      event.preventDefault();
      const form = event.target;
      const formData = new FormData(form);
      if (event.submitter && event.submitter.name) {{
        formData.set(event.submitter.name, event.submitter.value);
      }}
      const body = new URLSearchParams(formData);
      try {{
        await fetch('/api/action', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
          body,
        }});
      }} catch (error) {{
        document.getElementById('last-message').textContent =
          'ERROR: REST request failed: ' + error;
      }}
    }});

    const events = new EventSource('/events');
    events.onmessage = (event) => {{
      const data = JSON.parse(event.data);
      document.getElementById('last-message').textContent = data.last_message;
      const readiness = document.getElementById('readiness-text');
      readiness.textContent = data.gazebo_readiness_text;
      readiness.className = data.gazebo_ready ? 'ok' : 'bad';
      document.getElementById('readiness-summary').textContent =
        '(' + data.gazebo_readiness_summary + ')';
      document.getElementById('next-robot').textContent = data.next_robot_text;
      document.getElementById('delete-last-button').textContent = data.delete_last_text;
      document.getElementById('robot-forms').innerHTML = data.robot_forms_html;
      document.getElementById('robot-rows').innerHTML = data.robot_rows_html;
      document.getElementById('process-rows').innerHTML = data.process_rows_html;
    }};
  </script>
</body>
</html>""".format(
            message=message,
            readiness_class=readiness_class,
            readiness_text=readiness_text,
            readiness_summary=readiness_summary,
            next_name=html.escape(next_robot["name"]),
            next_x=next_robot["x"],
            next_y=next_robot["y"],
            next_z=next_robot["z"],
            next_yaw=next_robot["yaw"],
            last_robot=html.escape(last_robot_text),
            robot_forms=robot_forms_html,
            robot_rows=robot_rows_html,
            rows=rows_html,
        )

    def _render_robot_row(self, row):
        name = row["name"]
        form_id = "robot_%s" % name
        initial = row["initial"]
        command = row["command"]
        status = []
        status.append("gazebo" if row["in_gazebo"] else "not in gazebo")
        status.append("cmd running" if row["cmd_running"] else "cmd stopped")
        return """
        <tr>
          <td>
            {name}
            <input form="{form_id}" type="hidden" name="name" value="{name}">
          </td>
          <td>{status}</td>
          <td><input form="{form_id}" class="compact" name="init_x" value="{init_x:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_y" value="{init_y:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_z" value="{init_z:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="init_yaw" value="{init_yaw:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="cmd_x" value="{cmd_x:.3f}"></td>
          <td><input form="{form_id}" class="compact" name="cmd_yaw" value="{cmd_yaw:.3f}"></td>
          <td>
            <button form="{form_id}" name="action" value="start_cmd">Start</button>
            <button form="{form_id}" class="stop" name="action" value="stop_cmd">Stop</button>
          </td>
          <td>
            <button form="{form_id}" name="action" value="reset_robot_initial">Reset Pose</button>
          </td>
        </tr>""".format(
            form_id=html.escape(form_id),
            name=html.escape(name),
            status=html.escape(", ".join(status)),
            init_x=initial["x"],
            init_y=initial["y"],
            init_z=initial["z"],
            init_yaw=initial["yaw"],
            cmd_x=command["linear_x"],
            cmd_yaw=command["yaw_rate"],
        )

    def _send_html(self, body):
        encoded = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, payload):
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _get(self, form, key, default):
        value = form.get(key, [default])[0]
        return value if value != "" else default

    def _float(self, form, key, default):
        return float(self._get(form, key, str(default)))


def main():
    rospy.init_node("gazebo_session_webui")
    manager = GazeboSessionManager()
    RequestHandler.manager = manager
    server = ThreadingHTTPServer((manager.host, manager.port), RequestHandler)
    url = "http://%s:%d" % (manager.host, manager.port)
    print("\nGazebo Session Manager WebUI: %s\n" % url, flush=True)
    rospy.loginfo("Gazebo session web UI: %s", url)

    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()

    try:
        rospy.spin()
    finally:
        rospy.logwarn("Gazebo session manager is shutting down")
        server.shutdown()
        server.server_close()
        if manager.cleanup_on_shutdown:
            result = manager.cleanup_session("roslaunch shutdown")
            print("Gazebo Session Manager cleanup: %s" % result, flush=True)
            rospy.logwarn("Gazebo session cleanup: %s", result)


if __name__ == "__main__":
    main()
