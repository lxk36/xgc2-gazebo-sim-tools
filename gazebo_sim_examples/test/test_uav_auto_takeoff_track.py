#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import uav_auto_takeoff_track as auto  # noqa: E402


class FakePublisher:
    def __init__(self, connections):
        self.connections = connections

    def get_num_connections(self):
        return self.connections


class RunHarness(auto.AutoTakeoffTrack):
    def __init__(self, ready=True, altitude=True, tracking=True):
        self.args = SimpleNamespace(command_duration=3.0, post_takeoff_hold=1.0, height=3.0)
        self.ready = ready
        self.altitude = altitude
        self.tracking = tracking
        self.commands = []
        self.estimate = SimpleNamespace(position=SimpleNamespace(z=1.25))
        self.state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
        self.controller_state_name = "Custom1"
        self.custom1_seen = True
        self.successful_nmpc_samples = 2
        self.last_successful_nmpc_sequence = 2
        self.latest_nmpc_sample = SimpleNamespace(sequence=2, success=True, solver_status=0)

    def wait_ready(self):
        return self.ready

    def wait_altitude(self):
        return self.altitude

    def wait_tracking_active(self):
        return self.tracking

    def prepare_tracking_confirmation(self):
        pass

    def publish_command(self, command, duration):
        self.commands.append((command, duration))


class AutoTakeoffTrackTest(unittest.TestCase):
    def make_controller(self):
        controller = auto.AutoTakeoffTrack.__new__(auto.AutoTakeoffTrack)
        controller.args = SimpleNamespace(height=3.0, altitude_margin=0.25, max_settle_vz=0.6)
        controller.state = SimpleNamespace(connected=True, armed=False)
        controller.estimate = SimpleNamespace(
            position=SimpleNamespace(z=3.0), velocity=SimpleNamespace(z=0.1)
        )
        controller.command_pub = FakePublisher(1)
        controller.controller_state_name = None
        controller.latest_nmpc_sample = None
        controller.custom1_seen = False
        controller.successful_nmpc_samples = 0
        controller.last_successful_nmpc_sequence = 0
        controller.confirming_tracking = True
        return controller

    def test_readiness_requires_mavros_estimate_and_command_subscriber(self):
        controller = self.make_controller()
        self.assertTrue(controller.is_ready())

        controller.state.connected = False
        self.assertFalse(controller.is_ready())
        controller.state.connected = True

        controller.estimate.position.z = math.nan
        self.assertFalse(controller.is_ready())
        controller.estimate.position.z = 3.0

        controller.command_pub.connections = 0
        self.assertFalse(controller.is_ready())

    def test_altitude_requires_height_and_settled_vertical_velocity(self):
        controller = self.make_controller()
        self.assertTrue(controller.is_takeoff_altitude_reached())

        controller.estimate.position.z = 2.7
        self.assertFalse(controller.is_takeoff_altitude_reached())
        controller.estimate.position.z = 3.0

        controller.estimate.velocity.z = 0.7
        self.assertFalse(controller.is_takeoff_altitude_reached())

    def test_tracking_requires_custom1_two_good_solves_and_armed_connected_vehicle(self):
        controller = self.make_controller()
        controller.state.armed = True
        controller.custom1_seen = True
        controller.controller_state_name = "Custom1"
        controller.successful_nmpc_samples = 2
        self.assertTrue(controller.is_tracking_active())

        controller.successful_nmpc_samples = 1
        self.assertFalse(controller.is_tracking_active())
        controller.successful_nmpc_samples = 2

        controller.state.armed = False
        self.assertFalse(controller.is_tracking_active())
        controller.state.armed = True

        controller.state.connected = False
        self.assertFalse(controller.is_tracking_active())

    def test_tracking_becomes_inactive_when_controller_leaves_custom1(self):
        controller = self.make_controller()
        controller.state.armed = True
        controller._controller_status_cb(SimpleNamespace(data="Custom1"))
        controller.successful_nmpc_samples = 2
        self.assertTrue(controller.is_tracking_active())

        controller._controller_status_cb(SimpleNamespace(data="Hover"))
        self.assertFalse(controller.custom1_seen)
        self.assertFalse(controller.is_tracking_active())

    def test_nmpc_confirmation_counts_only_new_increasing_good_samples_after_custom1(self):
        controller = self.make_controller()

        controller._nmpc_debug_cb(SimpleNamespace(sequence=1, success=True, solver_status=0))
        self.assertEqual(0, controller.successful_nmpc_samples)

        controller._controller_status_cb(SimpleNamespace(data="Custom1"))
        controller._nmpc_debug_cb(SimpleNamespace(sequence=1, success=False, solver_status=0))
        controller._nmpc_debug_cb(SimpleNamespace(sequence=1, success=True, solver_status=1))
        self.assertEqual(0, controller.successful_nmpc_samples)

        controller._nmpc_debug_cb(SimpleNamespace(sequence=1, success=True, solver_status=0))
        controller._nmpc_debug_cb(SimpleNamespace(sequence=1, success=True, solver_status=0))
        self.assertEqual(1, controller.successful_nmpc_samples)

        controller._nmpc_debug_cb(SimpleNamespace(sequence=2, success=True, solver_status=0))
        self.assertEqual(2, controller.successful_nmpc_samples)
        self.assertEqual(2, controller.last_successful_nmpc_sequence)

    @mock.patch.object(auto.time, "sleep")
    @mock.patch.object(auto.rospy, "loginfo")
    def test_success_publishes_takeoff_then_custom1(self, _loginfo, _sleep):
        controller = RunHarness()
        self.assertEqual(auto.EXIT_SUCCESS, controller.run())
        self.assertEqual([("takeoff", 3.0), ("custom1", 3.0)], controller.commands)

    @mock.patch.object(auto.rospy, "logerr")
    def test_readiness_timeout_has_distinct_exit_code(self, _logerr):
        controller = RunHarness(ready=False)
        self.assertEqual(auto.EXIT_READY_TIMEOUT, controller.run())
        self.assertEqual([], controller.commands)

    @mock.patch.object(auto.rospy, "logerr")
    @mock.patch.object(auto.rospy, "loginfo")
    def test_takeoff_timeout_has_distinct_exit_code(self, _loginfo, _logerr):
        controller = RunHarness(altitude=False)
        self.assertEqual(auto.EXIT_TAKEOFF_TIMEOUT, controller.run())
        self.assertEqual([("takeoff", 3.0)], controller.commands)

    @mock.patch.object(auto.time, "sleep")
    @mock.patch.object(auto.rospy, "logerr")
    @mock.patch.object(auto.rospy, "loginfo")
    def test_tracking_timeout_has_distinct_exit_code(self, _loginfo, _logerr, _sleep):
        controller = RunHarness(tracking=False)
        self.assertEqual(auto.EXIT_TRACKING_TIMEOUT, controller.run())
        self.assertEqual([("takeoff", 3.0), ("custom1", 3.0)], controller.commands)


if __name__ == "__main__":
    unittest.main()
