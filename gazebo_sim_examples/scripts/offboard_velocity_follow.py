#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


def clamp(value, limit):
    return max(-limit, min(limit, value))


class OffboardVelocityFollow:
    def __init__(self):
        self.mavros_ns = rospy.get_param("~mavros_ns", "/uav1/mavros").rstrip("/")
        self.target_pose_topic = rospy.get_param("~target_pose_topic", "/vrpn_client_node/ugv1/pose")
        self.target_twist_topic = rospy.get_param("~target_twist_topic", "/vrpn_client_node/ugv1/twist")
        self.altitude_m = float(rospy.get_param("~altitude_m", 1.0))
        self.target_offset_x_m = float(rospy.get_param("~target_offset_x_m", 0.0))
        self.target_offset_y_m = float(rospy.get_param("~target_offset_y_m", 0.0))
        self.kp_xy = float(rospy.get_param("~kp_xy", 0.7))
        self.kp_z = float(rospy.get_param("~kp_z", 0.8))
        self.feedforward_target_velocity = bool(rospy.get_param("~feedforward_target_velocity", True))
        self.max_xy_speed_mps = float(rospy.get_param("~max_xy_speed_mps", 1.0))
        self.max_z_speed_mps = float(rospy.get_param("~max_z_speed_mps", 0.6))
        self.xy_deadband_m = float(rospy.get_param("~xy_deadband_m", 0.05))
        self.z_deadband_m = float(rospy.get_param("~z_deadband_m", 0.03))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.prestream_s = float(rospy.get_param("~prestream_s", 2.0))
        self.takeoff_hold_s = float(rospy.get_param("~takeoff_hold_s", 15.0))
        self.tracking_ramp_s = float(rospy.get_param("~tracking_ramp_s", 5.0))
        self.mode_timeout_s = float(rospy.get_param("~mode_timeout_s", 10.0))
        self.arm_timeout_s = float(rospy.get_param("~arm_timeout_s", 10.0))
        self.disarm_on_shutdown = bool(rospy.get_param("~disarm_on_shutdown", False))
        self.drive_ugv = bool(rospy.get_param("~drive_ugv", True))
        self.ugv_cmd_vel_topic = rospy.get_param("~ugv_cmd_vel_topic", "/ugv1/cmd_vel")
        self.ugv_linear_x_mps = float(rospy.get_param("~ugv_linear_x_mps", 1.0))
        self.ugv_angular_z_radps = float(rospy.get_param("~ugv_angular_z_radps", 1.0))

        self._lock = threading.Lock()
        self.state = None
        self.local_pose = None
        self.target_pose = None
        self.target_twist = None

        rospy.Subscriber(self.mavros_ns + "/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber(self.mavros_ns + "/local_position/pose", PoseStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber(self.target_pose_topic, PoseStamped, self._target_pose_cb, queue_size=10)
        rospy.Subscriber(self.target_twist_topic, TwistStamped, self._target_twist_cb, queue_size=10)
        self.velocity_pub = rospy.Publisher(
            self.mavros_ns + "/setpoint_velocity/cmd_vel",
            TwistStamped,
            queue_size=10,
        )
        self.ugv_cmd_pub = rospy.Publisher(self.ugv_cmd_vel_topic, Twist, queue_size=10)

        self.set_mode = rospy.ServiceProxy(self.mavros_ns + "/set_mode", SetMode)
        self.arming = rospy.ServiceProxy(self.mavros_ns + "/cmd/arming", CommandBool)

    def _state_cb(self, msg):
        with self._lock:
            self.state = msg

    def _pose_cb(self, msg):
        with self._lock:
            self.local_pose = msg

    def _target_pose_cb(self, msg):
        with self._lock:
            self.target_pose = msg

    def _target_twist_cb(self, msg):
        with self._lock:
            self.target_twist = msg

    def snapshot(self):
        with self._lock:
            return self.state, self.local_pose, self.target_pose, self.target_twist

    def wait_for_inputs(self):
        rospy.loginfo("waiting for MAVROS connection, local position, and target VRPN pose/twist")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_ugv_command()
            state, pose, target_pose, target_twist = self.snapshot()
            if state and state.connected and pose and target_pose and target_twist:
                rospy.loginfo("inputs ready: mode=%s armed=%s", state.mode, state.armed)
                return
            rate.sleep()

    def make_velocity_setpoint(self, blend=1.0, blend_from_x=None, blend_from_y=None):
        _state, pose, target_pose, target_twist = self.snapshot()
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        if pose is None or target_pose is None or target_twist is None:
            return msg

        px = pose.pose.position.x
        py = pose.pose.position.y
        pz = pose.pose.position.z
        tx = target_pose.pose.position.x + self.target_offset_x_m
        ty = target_pose.pose.position.y + self.target_offset_y_m
        blend = max(0.0, min(1.0, blend))
        if blend_from_x is not None and blend_from_y is not None:
            tx = blend_from_x + blend * (tx - blend_from_x)
            ty = blend_from_y + blend * (ty - blend_from_y)

        ex = tx - px
        ey = ty - py
        ez = self.altitude_m - pz

        vx = 0.0 if abs(ex) < self.xy_deadband_m else self.kp_xy * ex
        vy = 0.0 if abs(ey) < self.xy_deadband_m else self.kp_xy * ey
        vz = 0.0 if abs(ez) < self.z_deadband_m else self.kp_z * ez

        if self.feedforward_target_velocity:
            vx += blend * target_twist.twist.linear.x
            vy += blend * target_twist.twist.linear.y

        horizontal_speed = math.hypot(vx, vy)
        if horizontal_speed > self.max_xy_speed_mps > 0.0:
            scale = self.max_xy_speed_mps / horizontal_speed
            vx *= scale
            vy *= scale

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = clamp(vz, self.max_z_speed_mps)
        msg.twist.angular.z = 0.0
        return msg

    def make_ugv_command(self):
        msg = Twist()
        msg.linear.x = self.ugv_linear_x_mps
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.ugv_angular_z_radps
        return msg

    def publish_ugv_command(self):
        if self.drive_ugv:
            self.ugv_cmd_pub.publish(self.make_ugv_command())

    def publish_uav_setpoint(self, setpoint):
        self.velocity_pub.publish(setpoint)
        self.publish_ugv_command()

    def make_hold_setpoint(self, hold_x, hold_y):
        _state, pose, _target_pose, _target_twist = self.snapshot()
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        if pose is None:
            return msg

        ex = hold_x - pose.pose.position.x
        ey = hold_y - pose.pose.position.y
        ez = self.altitude_m - pose.pose.position.z

        vx = 0.0 if abs(ex) < self.xy_deadband_m else self.kp_xy * ex
        vy = 0.0 if abs(ey) < self.xy_deadband_m else self.kp_xy * ey
        vz = 0.0 if abs(ez) < self.z_deadband_m else self.kp_z * ez

        horizontal_speed = math.hypot(vx, vy)
        if horizontal_speed > self.max_xy_speed_mps > 0.0:
            scale = self.max_xy_speed_mps / horizontal_speed
            vx *= scale
            vy *= scale

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = clamp(vz, self.max_z_speed_mps)
        msg.twist.angular.z = 0.0
        return msg

    def takeoff_anchor(self):
        _state, pose, _target_pose, _target_twist = self.snapshot()
        if pose is None:
            return 0.0, 0.0
        return pose.pose.position.x, pose.pose.position.y

    def publish_for(self, duration_s, setpoint_fn):
        end_time = rospy.Time.now() + rospy.Duration(duration_s)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.publish_uav_setpoint(setpoint_fn())
            rate.sleep()

    def publish_tracking_ramp(self, duration_s, hold_x, hold_y):
        if duration_s <= 0.0:
            return
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration_s)
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("ramping from takeoff hold to VRPN tracking over %.1f s", duration_s)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            elapsed_s = (rospy.Time.now() - start_time).to_sec()
            blend = max(0.0, min(1.0, elapsed_s / duration_s))
            self.publish_uav_setpoint(
                self.make_velocity_setpoint(blend=blend, blend_from_x=hold_x, blend_from_y=hold_y)
            )
            rate.sleep()

    def request_offboard(self, setpoint_fn):
        rospy.loginfo("requesting OFFBOARD mode")
        deadline = rospy.Time.now() + rospy.Duration(self.mode_timeout_s)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_uav_setpoint(setpoint_fn())
            state, _pose, _target_pose, _target_twist = self.snapshot()
            if state and state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD mode accepted")
                return True
            try:
                self.set_mode(base_mode=0, custom_mode="OFFBOARD")
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(1.0, "set_mode OFFBOARD failed: %s", exc)
            rate.sleep()
        rospy.logerr("OFFBOARD mode was not accepted within %.1f s", self.mode_timeout_s)
        return False

    def request_arm(self, setpoint_fn):
        rospy.loginfo("requesting arm")
        deadline = rospy.Time.now() + rospy.Duration(self.arm_timeout_s)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_uav_setpoint(setpoint_fn())
            state, _pose, _target_pose, _target_twist = self.snapshot()
            if state and state.armed:
                rospy.loginfo("vehicle armed")
                return True
            try:
                self.arming(True)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(1.0, "arming failed: %s", exc)
            rate.sleep()
        rospy.logerr("vehicle did not arm within %.1f s", self.arm_timeout_s)
        return False

    def run(self):
        rospy.wait_for_service(self.mavros_ns + "/set_mode")
        rospy.wait_for_service(self.mavros_ns + "/cmd/arming")
        if self.drive_ugv:
            rospy.loginfo(
                "publishing UGV command on %s: linear.x=%.2f angular.z=%.2f",
                self.ugv_cmd_vel_topic,
                self.ugv_linear_x_mps,
                self.ugv_angular_z_radps,
            )
        self.wait_for_inputs()

        hold_x, hold_y = self.takeoff_anchor()
        hold_setpoint = lambda: self.make_hold_setpoint(hold_x, hold_y)
        rospy.loginfo(
            "takeoff hold target anchored at x=%.2f y=%.2f altitude=%.2f m",
            hold_x,
            hold_y,
            self.altitude_m,
        )

        rospy.loginfo("prestreaming velocity setpoints for %.1f s", self.prestream_s)
        self.publish_for(self.prestream_s, hold_setpoint)

        if not self.request_offboard(hold_setpoint):
            return 1
        if not self.request_arm(hold_setpoint):
            return 1

        rospy.loginfo("holding takeoff point for %.1f s before target tracking", self.takeoff_hold_s)
        self.publish_for(self.takeoff_hold_s, hold_setpoint)
        self.publish_tracking_ramp(self.tracking_ramp_s, hold_x, hold_y)

        rospy.loginfo(
            "tracking VRPN target pose=%s twist=%s at %.2f m altitude",
            self.target_pose_topic,
            self.target_twist_topic,
            self.altitude_m,
        )
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_uav_setpoint(self.make_velocity_setpoint())
            rate.sleep()
        return 0

    def shutdown(self):
        if not self.disarm_on_shutdown:
            return
        try:
            self.arming(False)
            rospy.loginfo("disarm requested on shutdown")
        except rospy.ServiceException as exc:
            rospy.logwarn("shutdown disarm failed: %s", exc)


def main():
    rospy.init_node("fs150_offboard_velocity_follow")
    node = OffboardVelocityFollow()
    rospy.on_shutdown(node.shutdown)
    raise SystemExit(node.run())


if __name__ == "__main__":
    main()
