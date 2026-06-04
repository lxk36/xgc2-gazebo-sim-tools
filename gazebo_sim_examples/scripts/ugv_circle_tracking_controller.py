#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class UgvCircleTrackingController:
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/ugv1/pose")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/ugv1/cmd_vel")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.pose_timeout_s = float(rospy.get_param("~pose_timeout_s", 0.5))

        self.radius_m = max(0.05, float(rospy.get_param("~radius_m", 2.0)))
        self.speed_mps = abs(float(rospy.get_param("~speed_mps", 1.0)))
        self.direction = 1.0 if float(rospy.get_param("~direction", 1.0)) >= 0.0 else -1.0
        self.lookahead_distance_m = max(
            0.1,
            float(rospy.get_param("~lookahead_distance_m", 0.6)),
        )
        self.center_from_initial_pose = bool(rospy.get_param("~center_from_initial_pose", True))
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))

        self.k_x = float(rospy.get_param("~k_x", 0.8))
        self.k_y = float(rospy.get_param("~k_y", 3.0))
        self.k_yaw = float(rospy.get_param("~k_yaw", 2.5))
        self.max_linear_speed_mps = abs(float(rospy.get_param("~max_linear_speed_mps", 1.5)))
        self.max_angular_speed_radps = abs(float(rospy.get_param("~max_angular_speed_radps", 3.0)))
        self.allow_reverse = bool(rospy.get_param("~allow_reverse", False))

        self._lock = threading.Lock()
        self._pose = None
        self._last_pose_time = rospy.Time(0)
        self._initialized = False
        self._start_time = rospy.Time(0)
        self._theta0 = 0.0

        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

    def _pose_cb(self, msg):
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        with self._lock:
            self._pose = msg
            self._last_pose_time = stamp

    def snapshot(self):
        with self._lock:
            return self._pose, self._last_pose_time

    def wait_for_pose(self):
        rospy.loginfo("waiting for UGV pose on %s", self.pose_topic)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose, _stamp = self.snapshot()
            if pose is not None:
                self.initialize_reference(pose)
                return True
            rate.sleep()
        return False

    def initialize_reference(self, pose):
        yaw = yaw_from_quaternion(pose.pose.orientation)
        x = pose.pose.position.x
        y = pose.pose.position.y

        if self.center_from_initial_pose:
            self._theta0 = wrap_pi(yaw - self.direction * 0.5 * math.pi)
            self.center_x = x - self.radius_m * math.cos(self._theta0)
            self.center_y = y - self.radius_m * math.sin(self._theta0)
        else:
            self._theta0 = math.atan2(y - self.center_y, x - self.center_x)

        self._start_time = rospy.Time.now()
        self._initialized = True
        rospy.loginfo(
            "UGV circle tracking: center=(%.2f, %.2f) radius=%.2f speed=%.2f direction=%+.0f cmd=%s",
            self.center_x,
            self.center_y,
            self.radius_m,
            self.speed_mps,
            self.direction,
            self.cmd_vel_topic,
        )

    def reference_at(self, x, y):
        theta = math.atan2(y - self.center_y, x - self.center_x)
        radial_error = math.hypot(x - self.center_x, y - self.center_y) - self.radius_m
        correction = math.atan(self.k_y * radial_error)
        yaw_ref = wrap_pi(theta + self.direction * (0.5 * math.pi + correction))
        omega = self.direction * self.speed_mps / self.radius_m
        x_ref = self.center_x + self.radius_m * math.cos(theta)
        y_ref = self.center_y + self.radius_m * math.sin(theta)
        return x_ref, y_ref, yaw_ref, self.speed_mps, omega, radial_error

    def make_command(self, pose, now):
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = yaw_from_quaternion(pose.pose.orientation)
        x_ref, y_ref, yaw_ref, v_ref, w_ref, radial_error = self.reference_at(x, y)
        target_distance = math.hypot(x_ref - x, y_ref - y)
        e_yaw = wrap_pi(yaw_ref - yaw)

        v_cmd = v_ref * max(0.0, math.cos(e_yaw))
        if not self.allow_reverse:
            v_cmd = max(0.15 * v_ref, v_cmd)
        w_cmd = w_ref + self.k_yaw * e_yaw

        if self.allow_reverse:
            v_cmd = clamp(v_cmd, -self.max_linear_speed_mps, self.max_linear_speed_mps)
        else:
            v_cmd = clamp(v_cmd, 0.0, self.max_linear_speed_mps)
        w_cmd = clamp(w_cmd, -self.max_angular_speed_radps, self.max_angular_speed_radps)

        msg = Twist()
        msg.linear.x = v_cmd
        msg.angular.z = w_cmd
        return msg, radial_error, target_distance, e_yaw

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def run(self):
        if not self.wait_for_pose():
            return 1

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            pose, stamp = self.snapshot()
            now = rospy.Time.now()
            if pose is None or (now - stamp).to_sec() > self.pose_timeout_s:
                rospy.logwarn_throttle(1.0, "UGV pose is stale; publishing zero cmd_vel")
                self.publish_stop()
                rate.sleep()
                continue

            command, radial_error, target_distance, e_yaw = self.make_command(pose, now)
            self.cmd_pub.publish(command)
            rospy.loginfo_throttle(
                1.0,
                "UGV circle error radial=%.2f lookahead=%.2f yaw=%.2f cmd=[%.2f %.2f]",
                radial_error,
                target_distance,
                e_yaw,
                command.linear.x,
                command.angular.z,
            )
            rate.sleep()
        return 0

    def shutdown(self):
        self.publish_stop()


def main():
    rospy.init_node("ugv_circle_tracking_controller")
    node = UgvCircleTrackingController()
    rospy.on_shutdown(node.shutdown)
    raise SystemExit(node.run())


if __name__ == "__main__":
    main()
