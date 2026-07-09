#!/usr/bin/env python3
import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path

try:
    from rigid_state_estimator_msgs.msg import PlanarStateEstimate
except ImportError:
    PlanarStateEstimate = None


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rmse(values):
    return math.sqrt(mean([value * value for value in values])) if values else 0.0


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
    return ordered[idx]


class NmpcRegressionMonitor:
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir", "/tmp/xgc2_gazebo_validation/nmpc")
        self.duration_s = float(rospy.get_param("~duration_s", 45.0))
        self.warmup_s = float(rospy.get_param("~warmup_s", 5.0))
        self.sample_rate_hz = float(rospy.get_param("~sample_rate_hz", 20.0))
        self.state_topic = rospy.get_param(
            "~state_topic", "/ugv1/alg/state_estimator/state"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/ugv1/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ugv1/cmd_vel")
        self.reference_path_topic = rospy.get_param(
            "~reference_path_topic",
            "/ugv1/alg/unicycle_reference_trajectory/visualization/reference_path",
        )
        self.predicted_path_topic = rospy.get_param(
            "~predicted_path_topic", "/ugv1/alg/nmpc/predicted_path"
        )
        self.metrics_path = os.path.join(self.output_dir, "metrics.json")
        self.csv_path = os.path.join(self.output_dir, "samples.csv")

        self._lock = threading.Lock()
        self._state = None
        self._odom = None
        self._cmd = None
        self._reference_path = None
        self._predicted_path_count = 0
        self._state_count = 0
        self._cmd_count = 0

        if PlanarStateEstimate is not None:
            rospy.Subscriber(self.state_topic, PlanarStateEstimate, self._state_cb, queue_size=50)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=50)
        rospy.Subscriber(self.reference_path_topic, Path, self._reference_path_cb, queue_size=2)
        rospy.Subscriber(self.predicted_path_topic, Path, self._predicted_path_cb, queue_size=10)

    def _state_cb(self, msg):
        with self._lock:
            self._state = msg
            self._state_count += 1

    def _odom_cb(self, msg):
        with self._lock:
            self._odom = msg

    def _cmd_cb(self, msg):
        with self._lock:
            self._cmd = msg
            self._cmd_count += 1

    def _reference_path_cb(self, msg):
        with self._lock:
            self._reference_path = msg

    def _predicted_path_cb(self, _msg):
        with self._lock:
            self._predicted_path_count += 1

    def snapshot(self):
        with self._lock:
            return (
                self._state,
                self._odom,
                self._cmd,
                self._reference_path,
                self._predicted_path_count,
                self._state_count,
                self._cmd_count,
            )

    @staticmethod
    def nearest_path_error(x, y, yaw, path):
        if path is None or not path.poses:
            return None, None
        best_dist = None
        best_yaw_err = None
        for pose_stamped in path.poses:
            pose = pose_stamped.pose
            dist = math.hypot(pose.position.x - x, pose.position.y - y)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_yaw_err = wrap_pi(yaw - yaw_from_quaternion(pose.orientation))
        return best_dist, best_yaw_err

    def wait_for_inputs(self):
        deadline = rospy.Time.now() + rospy.Duration(45.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state, odom, _cmd, reference_path, _pred_count, _state_count, _cmd_count = (
                self.snapshot()
            )
            pose_ready = state is not None or odom is not None
            if pose_ready and reference_path is not None and reference_path.poses:
                return
            rate.sleep()
        raise RuntimeError("timed out waiting for state/odom and reference path")

    def run(self):
        os.makedirs(self.output_dir, exist_ok=True)
        self.wait_for_inputs()
        start = rospy.Time.now()
        end = start + rospy.Duration(self.duration_s)
        warmup_end = start + rospy.Duration(self.warmup_s)
        rate = rospy.Rate(self.sample_rate_hz)
        rows = []

        while not rospy.is_shutdown() and rospy.Time.now() < end:
            now = rospy.Time.now()
            state, odom, cmd, reference_path, pred_count, state_count, cmd_count = self.snapshot()
            if state is not None:
                x = state.position.x
                y = state.position.y
                yaw = yaw_from_quaternion(state.orientation)
                estimator_state = getattr(state, "estimator_state", 0)
                estimator_flags = getattr(state, "flags", 0)
            elif odom is not None:
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                yaw = yaw_from_quaternion(odom.pose.pose.orientation)
                estimator_state = 0
                estimator_flags = 0
            else:
                rate.sleep()
                continue

            path_error, path_yaw_error = self.nearest_path_error(x, y, yaw, reference_path)
            if path_error is None:
                rate.sleep()
                continue

            rows.append(
                {
                    "t": (now - start).to_sec(),
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "path_error": path_error,
                    "path_yaw_error": path_yaw_error,
                    "cmd_linear": cmd.linear.x if cmd is not None else 0.0,
                    "cmd_angular": cmd.angular.z if cmd is not None else 0.0,
                    "predicted_path_count": pred_count,
                    "state_count": state_count,
                    "cmd_count": cmd_count,
                    "estimator_state": estimator_state,
                    "estimator_flags": estimator_flags,
                    "in_window": now >= warmup_end,
                }
            )
            rate.sleep()

        if not rows:
            raise RuntimeError("no NMPC regression samples collected")
        with open(self.csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        eval_rows = [row for row in rows if row["in_window"]]
        if not eval_rows:
            eval_rows = rows
        path_errors = [row["path_error"] for row in eval_rows]
        yaw_errors = [row["path_yaw_error"] for row in eval_rows]
        cmd_linear = [abs(row["cmd_linear"]) for row in eval_rows]
        cmd_angular = [abs(row["cmd_angular"]) for row in eval_rows]
        first_counts = rows[0]
        last_counts = rows[-1]
        metrics = {
            "duration_s": self.duration_s,
            "warmup_s": self.warmup_s,
            "sample_count": len(rows),
            "evaluated_sample_count": len(eval_rows),
            "state_topic": self.state_topic,
            "odom_topic": self.odom_topic,
            "cmd_topic": self.cmd_topic,
            "reference_path_topic": self.reference_path_topic,
            "predicted_path_topic": self.predicted_path_topic,
            "path_error_rmse_m": rmse(path_errors),
            "path_error_mean_m": mean(path_errors),
            "path_error_p95_m": percentile(path_errors, 0.95),
            "path_error_max_m": max(path_errors),
            "path_yaw_error_rmse_rad": rmse(yaw_errors),
            "path_yaw_error_mean_abs_rad": mean([abs(value) for value in yaw_errors]),
            "path_yaw_error_p95_abs_rad": percentile([abs(value) for value in yaw_errors], 0.95),
            "max_abs_cmd_linear_mps": max(cmd_linear) if cmd_linear else 0.0,
            "max_abs_cmd_angular_radps": max(cmd_angular) if cmd_angular else 0.0,
            "state_messages": last_counts["state_count"] - first_counts["state_count"],
            "cmd_messages": last_counts["cmd_count"] - first_counts["cmd_count"],
            "predicted_path_messages": last_counts["predicted_path_count"]
            - first_counts["predicted_path_count"],
            "final_estimator_state": rows[-1]["estimator_state"],
            "final_estimator_flags": rows[-1]["estimator_flags"],
            "csv_path": self.csv_path,
        }
        with open(self.metrics_path, "w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
        rospy.loginfo("wrote NMPC regression metrics to %s", self.metrics_path)
        rospy.loginfo(
            "NMPC regression: path_rmse=%.3f m path_p95=%.3f m yaw_rmse=%.3f rad "
            "cmd_msgs=%d predicted_msgs=%d",
            metrics["path_error_rmse_m"],
            metrics["path_error_p95_m"],
            metrics["path_yaw_error_rmse_rad"],
            metrics["cmd_messages"],
            metrics["predicted_path_messages"],
        )


def main():
    rospy.init_node("nmpc_tracking_regression_monitor")
    node = NmpcRegressionMonitor()
    node.run()


if __name__ == "__main__":
    main()
