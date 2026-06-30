#!/usr/bin/env python3
"""Publish RViz paths and TF for FS150 tracking scenes from VRPN output."""

import math
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

try:
    from multirotor_reference_trajectory.msg import AnalyticReference
except ImportError:
    AnalyticReference = None


def finite(value):
    return math.isfinite(float(value))


def param_at(msg, index, default):
    return msg.params[index] if index < len(msg.params) and finite(msg.params[index]) else default


def safe_radius(value):
    if not finite(value) or abs(value) < 1.0e-6:
        return 1.0
    return abs(value)


def default_duration(analytic_type):
    if AnalyticReference is None:
        return 60.0
    if analytic_type == AnalyticReference.ANALYTIC_LINE:
        return 8.0
    if analytic_type == AnalyticReference.ANALYTIC_LEMNISCATE:
        return 20.0
    if analytic_type in (AnalyticReference.ANALYTIC_HELIX_YZ, AnalyticReference.ANALYTIC_HELIX_XY):
        return 25.0
    if analytic_type == AnalyticReference.ANALYTIC_TORUS_KNOT:
        return 35.0
    return 60.0


def origin_tuple(msg):
    return (msg.origin.position.x, msg.origin.position.y, msg.origin.position.z)


def add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale3(value, scalar):
    return (value[0] * scalar, value[1] * scalar, value[2] * scalar)


def line_point(msg, t, duration):
    start = origin_tuple(msg)
    target = (
        param_at(msg, 0, start[0] + 1.0),
        param_at(msg, 1, start[1] + 1.0),
        param_at(msg, 2, start[2] + 1.0),
    )
    target_v = (param_at(msg, 3, 0.0), param_at(msg, 4, 0.0), param_at(msg, 5, 0.0))
    start_v = (param_at(msg, 6, 0.0), param_at(msg, 7, 0.0), param_at(msg, 8, 0.0))
    t_final = max(1.0e-3, duration)
    a_delta = sub3(sub3(target, start), scale3(start_v, t_final))
    d_delta = sub3(start_v, target_v)
    a0 = start
    a1 = start_v
    a3 = scale3(add3(scale3(a_delta, 10.0), scale3(d_delta, 4.0 * t_final)), 1.0 / (t_final ** 3))
    a4 = scale3(add3(scale3(a_delta, -15.0), scale3(d_delta, -7.0 * t_final)), 1.0 / (t_final ** 4))
    a5 = scale3(add3(scale3(a_delta, 6.0), scale3(d_delta, 3.0 * t_final)), 1.0 / (t_final ** 5))
    return add3(add3(add3(add3(a0, scale3(a1, t)), scale3(a3, t ** 3)), scale3(a4, t ** 4)), scale3(a5, t ** 5))


def circle_point(msg, t, height_axis):
    radius = safe_radius(param_at(msg, 0, 3.0))
    line_speed = max(0.0, param_at(msg, 1, 3.0))
    height = param_at(msg, 2, 3.0)
    z_amplitude = param_at(msg, 3, 1.0) if height_axis else 0.0
    z_frequency = param_at(msg, 4, 0.5)
    center_x = param_at(msg, 6, 0.0)
    center_y = param_at(msg, 7, 0.0)
    omega = line_speed / radius
    return (
        center_x + radius * math.cos(omega * t),
        center_y + radius * math.sin(omega * t),
        height + z_amplitude * math.sin(z_frequency * t),
    )


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def analytic_point(msg, t, duration):
    if AnalyticReference is None:
        return origin_tuple(msg)
    analytic_type = msg.analytic_type
    origin = origin_tuple(msg)
    if analytic_type == AnalyticReference.ANALYTIC_HOLD:
        return (origin[0], origin[1], param_at(msg, 2, 3.0))
    if analytic_type == AnalyticReference.ANALYTIC_CIRCLE:
        return circle_point(msg, t, False)
    if analytic_type == AnalyticReference.ANALYTIC_HEIGHT_CIRCLE:
        return circle_point(msg, t, True)
    if analytic_type == AnalyticReference.ANALYTIC_CIRCLE_ENTRY:
        entry = max(0.0, param_at(msg, 5, 5.0))
        if t >= entry:
            return circle_point(msg, t - entry, True)
        start = (origin[0], origin[1], param_at(msg, 2, 3.0))
        end = circle_point(msg, 0.0, True)
        blend = smoothstep(t / max(entry, 1.0e-3))
        return add3(start, scale3(sub3(end, start), blend))
    if analytic_type == AnalyticReference.ANALYTIC_FIGURE_EIGHT:
        radius = safe_radius(param_at(msg, 0, 3.0))
        line_speed = max(0.0, param_at(msg, 1, 3.0))
        height = param_at(msg, 2, 3.0)
        omega = line_speed / radius
        wt = omega * t
        return (origin[0] + radius * math.sin(wt), origin[1] + 0.5 * radius * math.sin(2.0 * wt), height)
    if analytic_type == AnalyticReference.ANALYTIC_LINE:
        return line_point(msg, t, duration)
    if analytic_type == AnalyticReference.ANALYTIC_LEMNISCATE:
        radius = safe_radius(param_at(msg, 0, 1.0))
        omega = abs(param_at(msg, 1, 0.9)) or 0.9
        height = param_at(msg, 2, 1.0)
        wt = omega * t
        return add3(origin, (radius * math.sin(wt), radius * math.sin(wt) * math.cos(wt), height))
    if analytic_type == AnalyticReference.ANALYTIC_HELIX_YZ:
        radius = safe_radius(param_at(msg, 0, 1.0))
        omega = abs(param_at(msg, 1, 1.5)) or 1.5
        linear_scale = max(1.0e-3, abs(param_at(msg, 2, 10.0)))
        wt = omega * t
        return add3(origin, (t / linear_scale, radius * math.cos(wt), radius * math.sin(wt)))
    if analytic_type == AnalyticReference.ANALYTIC_HELIX_XY:
        radius = safe_radius(param_at(msg, 0, 1.0))
        omega = abs(param_at(msg, 1, 0.9)) or 0.9
        linear_scale = max(1.0e-3, abs(param_at(msg, 2, 10.0)))
        wt = omega * t
        return add3(origin, (radius * math.cos(wt), radius * math.sin(wt), t / linear_scale))
    if analytic_type == AnalyticReference.ANALYTIC_TORUS_KNOT:
        omega = abs(param_at(msg, 0, 0.9)) or 0.9
        scale = abs(param_at(msg, 1, 0.3)) or 0.3
        wt = omega * t
        return add3(
            origin,
            (
                scale * (math.sin(wt) + 2.0 * math.sin(2.0 * wt)),
                scale * (math.cos(wt) - 2.0 * math.cos(2.0 * wt)),
                scale * (4.0 + math.sin(3.0 * wt)),
            ),
        )
    return origin


class Fs150TrackingVisualizer:
    def __init__(self):
        ns = rospy.get_param("~ns", "uav1")
        tracker = rospy.get_param("~tracker", ns)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.model_frame = rospy.get_param("~model_frame", f"{ns}_base_link")
        self.max_path_points = int(rospy.get_param("~max_path_points", 3000))
        self.preview_dt = float(rospy.get_param("~preview_dt", 0.1))
        self.preview_max_points = int(rospy.get_param("~preview_max_points", 2000))
        vrpn_pose_topic = rospy.get_param("~vrpn_pose_topic", f"/vrpn_client_node/{tracker}/pose")
        tracking_error_topic = rospy.get_param("~tracking_error_topic", f"/{ns}/alg/tracking/position_error")
        active_analytic_topic = rospy.get_param(
            "~active_analytic_topic", f"/{ns}/alg/multirotor_reference_trajectory/active/analytic"
        )

        self.actual_points = deque(maxlen=self.max_path_points)
        self.reference_points = deque(maxlen=self.max_path_points)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.actual_path_pub = rospy.Publisher(f"/{ns}/visualization/actual_path", Path, queue_size=1)
        self.reference_path_pub = rospy.Publisher(f"/{ns}/visualization/reference_path", Path, queue_size=1)
        self.preview_path_pub = rospy.Publisher(f"/{ns}/visualization/reference_preview", Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(f"/{ns}/visualization/reference_marker", Marker, queue_size=1)
        rospy.Subscriber(vrpn_pose_topic, PoseStamped, self.vrpn_pose_callback, queue_size=20)
        rospy.Subscriber(tracking_error_topic, Float32MultiArray, self.tracking_error_callback, queue_size=20)
        if AnalyticReference is not None:
            rospy.Subscriber(active_analytic_topic, AnalyticReference, self.active_analytic_callback, queue_size=1)
        else:
            rospy.logwarn("multirotor_reference_trajectory messages are unavailable; reference preview disabled")

    def make_pose(self, xyz, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0
        return pose

    def publish_path(self, publisher, points, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        path.poses = list(points)
        publisher.publish(path)

    def vrpn_pose_callback(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose
        self.actual_points.append(pose)
        self.publish_path(self.actual_path_pub, self.actual_points, stamp)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.model_frame
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def tracking_error_callback(self, msg):
        if len(msg.data) < 7:
            return
        stamp = rospy.Time.now()
        xyz = (float(msg.data[4]), float(msg.data[5]), float(msg.data[6]))
        if not all(finite(value) for value in xyz):
            return
        pose = self.make_pose(xyz, stamp)
        self.reference_points.append(pose)
        self.publish_path(self.reference_path_pub, self.reference_points, stamp)
        self.publish_reference_marker(xyz, stamp)

    def active_analytic_callback(self, msg):
        duration = msg.duration if msg.duration > 0.0 and finite(msg.duration) else default_duration(msg.analytic_type)
        duration = max(0.0, duration)
        dt = max(1.0e-3, self.preview_dt)
        count = int(duration / dt) + 1
        if count > self.preview_max_points:
            count = self.preview_max_points
            dt = duration / max(1, count - 1)
        stamp = rospy.Time.now()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        for idx in range(max(1, count)):
            t = min(duration, idx * dt)
            path.poses.append(self.make_pose(analytic_point(msg, t, duration), stamp))
        self.preview_path_pub.publish(path)

    def publish_reference_marker(self, xyz, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "fs150_reference"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(*xyz)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.42
        marker.color.b = 0.05
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main():
    rospy.init_node("fs150_vrpn_tracking_visualizer")
    Fs150TrackingVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
