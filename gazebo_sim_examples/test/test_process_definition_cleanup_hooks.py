#!/usr/bin/env python3

import concurrent.futures
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MANIFEST_PATH = PACKAGE_DIR / "process-definitions" / "xgc2-gazebo-sim-tools.json"

FAKE_ROSSERVICE = r'''#!/usr/bin/env python3
import fcntl
import json
import os
import sys
import time

if os.environ.get("XGC_CLEANUP_TEST_FAIL") == "true":
    raise SystemExit(7)

state_dir = os.environ["XGC_CLEANUP_TEST_STATE"]
counter_path = os.path.join(state_dir, "counter.json")
lock_path = os.path.join(state_dir, "counter.lock")
with open(lock_path, "a+") as state_lock:
    fcntl.flock(state_lock.fileno(), fcntl.LOCK_EX)
    try:
        with open(counter_path) as source:
            counter = json.load(source)
    except FileNotFoundError:
        counter = {"active": 0, "maximum": 0}
    counter["active"] += 1
    counter["maximum"] = max(counter["maximum"], counter["active"])
    with open(counter_path, "w") as target:
        json.dump(counter, target)
    fcntl.flock(state_lock.fileno(), fcntl.LOCK_UN)

time.sleep(float(os.environ.get("XGC_CLEANUP_TEST_DELAY", "0.08")))

with open(lock_path, "a+") as state_lock:
    fcntl.flock(state_lock.fileno(), fcntl.LOCK_EX)
    with open(counter_path) as source:
        counter = json.load(source)
    counter["active"] -= 1
    with open(counter_path, "w") as target:
        json.dump(counter, target)
'''


class CleanupHookManifestTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with MANIFEST_PATH.open() as source:
            cls.manifest = json.load(source)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.fake_rosservice = Path(self.temporary.name) / "rosservice"
        self.fake_rosservice.write_text(FAKE_ROSSERVICE)
        self.fake_rosservice.chmod(0o700)
        self.port_seed = 900000000 + os.getpid()

    @classmethod
    def definition(cls, definition_id, version):
        matches = [
            definition
            for definition in cls.manifest["definitions"]
            if definition["id"] == definition_id and definition["version"] == version
        ]
        if len(matches) != 1:
            raise AssertionError(
                "expected one {}@{}, found {}".format(definition_id, version, len(matches))
            )
        return matches[0]

    @staticmethod
    def lock_path(port):
        return Path(
            "/tmp/xgc2-gazebo-delete-model-u{}-ros-p{}.lock".format(os.getuid(), port)
        )

    def cleanup_lock(self, port):
        path = self.lock_path(port)
        if path.exists() or path.is_symlink():
            path.unlink()

    def invoke(self, hook, model_name, port, extra_environment=None):
        arguments = hook["command"]["args"]
        environment = os.environ.copy()
        environment["XGC_CLEANUP_TEST_STATE"] = self.temporary.name
        if extra_environment:
            environment.update(extra_environment)
        return subprocess.run(
            [
                sys.executable,
                "-c",
                arguments[1],
                str(self.fake_rosservice),
                model_name,
                str(port),
                arguments[-1],
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=environment,
            timeout=10,
            check=False,
        )

    def test_new_versions_preserve_old_versions_and_define_expected_hooks(self):
        integrated_old = self.definition("px4-multirotor-nmpc-robot", "1.3.0")
        sitl_old = self.definition("px4-sitl-fs150", "1.3.0")
        integrated = self.definition("px4-multirotor-nmpc-robot", "1.4.0")
        sitl = self.definition("px4-sitl-fs150", "1.4.0")

        self.assertEqual(integrated_old["beforeStart"]["timeout"], 5_000_000_000)
        self.assertEqual(integrated_old["beforeStop"]["timeout"], 5_000_000_000)
        self.assertEqual(sitl_old["beforeStop"]["timeout"], 5_000_000_000)
        self.assertEqual(integrated["beforeStart"]["timeout"], 45_000_000_000)
        self.assertEqual(integrated["beforeStop"]["timeout"], 45_000_000_000)
        self.assertEqual(sitl["beforeStop"]["timeout"], 45_000_000_000)
        self.assertNotIn("beforeStart", sitl)
        canonical_script = integrated["beforeStart"]["command"]["args"][1]
        self.assertEqual(integrated["beforeStop"]["command"]["args"][1], canonical_script)
        self.assertEqual(sitl["beforeStop"]["command"]["args"][1], canonical_script)

        for hook, mode in (
            (integrated["beforeStart"], "strict"),
            (integrated["beforeStop"], "strict"),
            (sitl["beforeStop"], "best-effort"),
        ):
            command = hook["command"]
            self.assertEqual(command["executable"], "/usr/bin/python3")
            self.assertTrue(command["directExecutable"])
            self.assertEqual(
                command["args"][2:],
                [
                    "/opt/ros/noetic/bin/rosservice",
                    "${modelName}",
                    "${rosMasterPort}",
                    mode,
                ],
            )
            script = command["args"][1]
            for required in (
                "rosservice,model,raw_port,mode=sys.argv[1:]",
                "port=int(raw_port)",
                "os.O_NOFOLLOW | os.O_CLOEXEC",
                "os.open(lock_path,os.O_RDWR | os.O_CREAT",
                "os.fstat(fd)",
                "stat.S_ISREG",
                "info.st_uid != uid",
                "fcntl.flock(fd,fcntl.LOCK_EX)",
                "fcntl.flock(fd,fcntl.LOCK_UN)",
                "timeout=4",
                "check=False",
            ):
                self.assertIn(required, script)
            self.assertNotIn("shell=True", script)

    def test_gazebo_server_presets_system_plugin_dependency_path(self):
        legacy = self.definition("gazebo-server", "3.7.0")
        previous = self.definition("gazebo-server", "3.8.0")
        current = self.definition("gazebo-server", "3.9.0")
        plugin_path = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"

        self.assertNotIn(plugin_path, legacy["command"]["env"]["LD_LIBRARY_PATH"].split(":"))
        for definition in (previous, current):
            for variable in ("GAZEBO_PLUGIN_PATH", "LD_LIBRARY_PATH"):
                paths = definition["command"]["env"][variable].split(":")
                self.assertIn(plugin_path, paths)
                self.assertEqual(paths.count(plugin_path), 1)

        properties = current["parameters"]["properties"]
        self.assertTrue(properties["overrideWorldPhysicsTiming"]["default"])
        self.assertEqual(properties["maxStepSize"]["default"], 0.004)
        self.assertEqual(properties["realTimeUpdateRate"]["default"], 250)

    def test_same_ros_master_port_serializes_ten_cleanup_calls(self):
        hook = self.definition("px4-multirotor-nmpc-robot", "1.4.0")["beforeStop"]
        port = self.port_seed
        self.addCleanup(self.cleanup_lock, port)
        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
            results = list(
                executor.map(lambda index: self.invoke(hook, "uav{}".format(index), port), range(10))
            )
        self.assertTrue(all(result.returncode == 0 for result in results))
        with (Path(self.temporary.name) / "counter.json").open() as source:
            counter = json.load(source)
        self.assertEqual(counter, {"active": 0, "maximum": 1})

    def test_different_ros_master_ports_can_cleanup_concurrently(self):
        hook = self.definition("px4-multirotor-nmpc-robot", "1.4.0")["beforeStop"]
        ports = [self.port_seed + index + 1 for index in range(10)]
        for port in ports:
            self.addCleanup(self.cleanup_lock, port)
        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
            results = list(
                executor.map(
                    lambda item: self.invoke(hook, "uav{}".format(item[0]), item[1]),
                    enumerate(ports),
                )
            )
        self.assertTrue(all(result.returncode == 0 for result in results))
        with (Path(self.temporary.name) / "counter.json").open() as source:
            counter = json.load(source)
        self.assertEqual(counter["active"], 0)
        self.assertGreater(counter["maximum"], 1)

    def test_integrated_cleanup_is_strict_and_sitl_cleanup_is_best_effort(self):
        integrated = self.definition("px4-multirotor-nmpc-robot", "1.4.0")["beforeStop"]
        sitl = self.definition("px4-sitl-fs150", "1.4.0")["beforeStop"]
        strict_port = self.port_seed + 20
        best_effort_port = self.port_seed + 21
        self.addCleanup(self.cleanup_lock, strict_port)
        self.addCleanup(self.cleanup_lock, best_effort_port)
        environment = {"XGC_CLEANUP_TEST_FAIL": "true"}
        self.assertNotEqual(
            self.invoke(integrated, "uav-strict", strict_port, environment).returncode, 0
        )
        self.assertEqual(
            self.invoke(sitl, "uav-best-effort", best_effort_port, environment).returncode, 0
        )

    def test_lock_open_failure_obeys_strict_and_best_effort_policy(self):
        integrated = self.definition("px4-multirotor-nmpc-robot", "1.4.0")["beforeStop"]
        sitl = self.definition("px4-sitl-fs150", "1.4.0")["beforeStop"]
        strict_port = self.port_seed + 30
        best_effort_port = self.port_seed + 31
        for port in (strict_port, best_effort_port):
            lock_path = self.lock_path(port)
            lock_path.symlink_to(Path(self.temporary.name) / (str(port) + "-target"))
            self.addCleanup(self.cleanup_lock, port)
        self.assertNotEqual(self.invoke(integrated, "uav-strict", strict_port).returncode, 0)
        self.assertEqual(self.invoke(sitl, "uav-best-effort", best_effort_port).returncode, 0)


if __name__ == "__main__":
    unittest.main()
