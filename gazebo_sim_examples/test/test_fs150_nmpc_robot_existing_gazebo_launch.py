#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


LAUNCH_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch",
    "fs150_nmpc_robot_existing_gazebo.launch",
)


class ExistingGazeboLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(LAUNCH_PATH).getroot()
        with open(LAUNCH_PATH, "r", encoding="utf-8") as stream:
            cls.source = stream.read()

    def test_exposes_per_robot_identity_pose_and_port_contract(self):
        args = {element.attrib["name"]: element for element in self.root.findall("./arg")}
        required = {
            "ns",
            "model_name",
            "vrpn_rigid_body",
            "mav_system_id",
            "px4_instance",
            "x",
            "y",
            "z",
            "yaw",
            "mavlink_tcp_port",
            "mavlink_udp_port",
            "qgc_udp_port",
            "sdk_udp_port",
            "mavros_local_port",
            "mavros_remote_port",
            "work_dir",
        }
        self.assertEqual(set(), required.difference(args))
        self.assertIn("model_name", args["work_dir"].attrib["default"])
        self.assertIn("px4_instance", args["mavlink_tcp_port"].attrib["default"])
        self.assertIn("px4_instance", args["mavros_remote_port"].attrib["default"])

    def test_shared_gazebo_startup_is_not_publicly_enableable(self):
        self.assertIsNone(self.root.find("./arg[@name='start_gazebo']"))
        fs150_include = next(
            include
            for include in self.root.findall("./include")
            if include.attrib.get("file") == "$(arg fs150_launch)"
        )
        include_args = {
            element.attrib["name"]: element.attrib.get("value")
            for element in fs150_include.findall("./arg")
        }
        self.assertEqual("false", include_args.get("start_gazebo"))

        forbidden = (
            "empty_world.launch",
            "gazebo_sim_vrpn_bridge",
            "vrpn_client_ros",
            "gzserver",
            "gzclient",
        )
        for token in forbidden:
            self.assertNotIn(token, self.source)

    def test_starts_complete_per_robot_nmpc_stack(self):
        include_files = {include.attrib.get("file", "") for include in self.root.findall("./include")}
        expected_fragments = {
            "estimator_vrpn_px4_rotor_state",
            "hover_thrust_estimator",
            "px4_multirotor_controller",
            "multirotor_reference_trajectory",
        }
        for fragment in expected_fragments:
            self.assertTrue(
                any(fragment in include_file for include_file in include_files),
                "missing include for %s" % fragment,
            )


if __name__ == "__main__":
    unittest.main()
