#!/usr/bin/env python3
"""Drive the PX4 multirotor controller through takeoff into trajectory tracking."""

import argparse
import math
import sys
import time

import rospy
from mavros_msgs.msg import State
from px4_multirotor_controller_msgs.msg import NmpcDebugSample
from rigid_state_estimator_msgs.msg import RigidStateEstimate
from std_msgs.msg import String


EXIT_SUCCESS = 0
EXIT_READY_TIMEOUT = 2
EXIT_TAKEOFF_TIMEOUT = 3
EXIT_TRACKING_TIMEOUT = 4


class AutoTakeoffTrack:
    def __init__(self, args):
        self.args = args
        self.state = None
        self.estimate = None
        self.controller_state_name = None
        self.latest_nmpc_sample = None
        self.custom1_seen = False
        self.successful_nmpc_samples = 0
        self.last_successful_nmpc_sequence = 0
        self.confirming_tracking = False
        self.command_pub = rospy.Publisher(args.command_topic, String, queue_size=10)
        rospy.Subscriber(f"/{args.ns}/mavros/state", State, self._state_cb, queue_size=1)
        rospy.Subscriber(
            f"/{args.ns}/alg/state_estimator/state",
            RigidStateEstimate,
            self._estimate_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            f"/{args.ns}/custom/statustext",
            String,
            self._controller_status_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            f"/{args.ns}/alg/nmpc/debug_sample",
            NmpcDebugSample,
            self._nmpc_debug_cb,
            queue_size=1,
        )

    def _state_cb(self, msg):
        self.state = msg

    def _estimate_cb(self, msg):
        self.estimate = msg

    def _controller_status_cb(self, msg):
        self.controller_state_name = msg.data
        if self.confirming_tracking:
            self.custom1_seen = msg.data == "Custom1"

    def _nmpc_debug_cb(self, msg):
        self.latest_nmpc_sample = msg
        if not self.confirming_tracking or not self.custom1_seen:
            return
        if (
            msg.sequence > 0
            and msg.sequence > self.last_successful_nmpc_sequence
            and msg.success
            and msg.solver_status == 0
        ):
            self.last_successful_nmpc_sequence = msg.sequence
            self.successful_nmpc_samples += 1

    def is_ready(self):
        connected = self.state is not None and self.state.connected
        estimate_ok = self.estimate is not None and math.isfinite(self.estimate.position.z)
        command_ok = self.command_pub.get_num_connections() > 0
        return connected and estimate_ok and command_ok

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.is_ready():
                return True
            time.sleep(0.1)
        return False

    def publish_command(self, command, duration):
        msg = String(data=command)
        deadline = time.monotonic() + duration
        interval = 1.0 / self.args.command_rate
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.command_pub.publish(msg)
            time.sleep(interval)

    def is_takeoff_altitude_reached(self):
        if self.estimate is None or not math.isfinite(self.estimate.position.z):
            return False
        z = self.estimate.position.z
        vz = self.estimate.velocity.z if math.isfinite(self.estimate.velocity.z) else 0.0
        return z >= self.args.height - self.args.altitude_margin and abs(vz) <= self.args.max_settle_vz

    def wait_altitude(self):
        deadline = time.monotonic() + self.args.takeoff_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.is_takeoff_altitude_reached():
                return True
            if self.state is not None and not self.state.armed:
                self.publish_command("takeoff", 0.5)
            time.sleep(0.1)
        return False

    def prepare_tracking_confirmation(self):
        self.controller_state_name = None
        self.latest_nmpc_sample = None
        self.custom1_seen = False
        self.successful_nmpc_samples = 0
        self.last_successful_nmpc_sequence = 0
        self.confirming_tracking = True

    def is_tracking_active(self):
        vehicle_ready = self.state is not None and self.state.connected and self.state.armed
        return (
            vehicle_ready
            and self.custom1_seen
            and self.controller_state_name == "Custom1"
            and self.successful_nmpc_samples >= 2
        )

    def wait_tracking_active(self):
        deadline = time.monotonic() + self.args.tracking_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.is_tracking_active():
                return True
            time.sleep(0.1)
        return False

    def run(self):
        if not self.wait_ready():
            rospy.logerr(
                "[uav_auto] timed out waiting for MAVROS connection, a finite state estimate, "
                "and a /command subscriber"
            )
            return EXIT_READY_TIMEOUT

        rospy.loginfo("[uav_auto] publishing takeoff")
        self.publish_command("takeoff", self.args.command_duration)

        if not self.wait_altitude():
            z = self.estimate.position.z if self.estimate is not None else float("nan")
            rospy.logerr(
                "[uav_auto] takeoff altitude not reached: z=%.3f target=%.3f",
                z,
                self.args.height,
            )
            return EXIT_TAKEOFF_TIMEOUT

        time.sleep(self.args.post_takeoff_hold)
        self.prepare_tracking_confirmation()
        rospy.loginfo("[uav_auto] publishing custom1")
        self.publish_command("custom1", self.args.command_duration)

        if not self.wait_tracking_active():
            connected = self.state is not None and self.state.connected
            armed = self.state is not None and self.state.armed
            mode = self.state.mode if self.state is not None else "unknown"
            sequence = self.latest_nmpc_sample.sequence if self.latest_nmpc_sample is not None else 0
            solver_success = (
                self.latest_nmpc_sample.success if self.latest_nmpc_sample is not None else False
            )
            solver_status = (
                self.latest_nmpc_sample.solver_status if self.latest_nmpc_sample is not None else -1
            )
            rospy.logerr(
                "[uav_auto] tracking did not become active: connected=%s armed=%s mode=%s "
                "controller_state=%s successful_nmpc_samples=%d sequence=%d "
                "solver_success=%s solver_status=%d",
                connected,
                armed,
                mode,
                self.controller_state_name or "unknown",
                self.successful_nmpc_samples,
                sequence,
                solver_success,
                solver_status,
            )
            return EXIT_TRACKING_TIMEOUT

        rospy.loginfo(
            "[uav_auto] NMPC tracking confirmed: sequence=%d successful_samples=%d mode=%s",
            self.last_successful_nmpc_sequence,
            self.successful_nmpc_samples,
            self.state.mode,
        )
        rospy.loginfo("[uav_auto] auto takeoff-track command sequence complete")
        return EXIT_SUCCESS


def parse_args():
    parser = argparse.ArgumentParser(description="Auto command UAV takeoff then custom1 tracking.")
    parser.add_argument("--ns", default="uav1")
    parser.add_argument("--height", type=float, default=3.0)
    parser.add_argument("--command-topic", default="/command")
    parser.add_argument("--ready-timeout", type=float, default=60.0)
    parser.add_argument("--takeoff-timeout", type=float, default=60.0)
    parser.add_argument("--tracking-timeout", type=float, default=60.0)
    parser.add_argument("--altitude-margin", type=float, default=0.25)
    parser.add_argument("--max-settle-vz", type=float, default=0.6)
    parser.add_argument("--post-takeoff-hold", type=float, default=1.0)
    parser.add_argument("--command-rate", type=float, default=2.0)
    parser.add_argument("--command-duration", type=float, default=3.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("uav_auto_takeoff_track", anonymous=True, disable_signals=True)
    return AutoTakeoffTrack(args).run()


if __name__ == "__main__":
    sys.exit(main())
