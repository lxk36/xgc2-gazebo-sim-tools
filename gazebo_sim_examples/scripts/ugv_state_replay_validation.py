#!/usr/bin/env python3
import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite(value, default=0.0):
    return value if math.isfinite(value) else default


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


class ReplayValidator:
    def __init__(self):
        self.dataset_path = rospy.get_param("~dataset_path")
        self.output_dir = rospy.get_param("~output_dir", "/tmp/xgc2_gazebo_validation/replay")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ugv1/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/ugv1/odom")
        self.linear_col = int(rospy.get_param("~linear_col", 7)) - 1
        self.angular_col = int(rospy.get_param("~angular_col", 8)) - 1
        self.linear_scale = float(rospy.get_param("~linear_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))
        self.max_samples = int(rospy.get_param("~max_samples", 0))
        self.start_delay_s = float(rospy.get_param("~start_delay_s", 1.0))
        self.stop_after_s = float(rospy.get_param("~stop_after_s", 1.0))
        self.compare_relative = bool(rospy.get_param("~compare_relative", True))
        self.metrics_path = os.path.join(self.output_dir, "metrics.json")
        self.csv_path = os.path.join(self.output_dir, "samples.csv")

        self._lock = threading.Lock()
        self._odom = None
        self._odom_stamp = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)

    def _odom_cb(self, msg):
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        with self._lock:
            self._odom = msg
            self._odom_stamp = stamp

    def odom_snapshot(self):
        with self._lock:
            return self._odom, self._odom_stamp

    def load_dataset(self):
        rows = []
        with open(self.dataset_path, "r", encoding="utf-8") as handle:
            for line in handle:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                values = [float(part) for part in stripped.split()]
                if len(values) <= max(7, self.linear_col, self.angular_col):
                    raise ValueError("dataset row has fewer columns than requested inputs")
                rows.append(values)
                if self.max_samples > 0 and len(rows) >= self.max_samples:
                    break
        if len(rows) < 2:
            raise ValueError("dataset must contain at least two samples")
        return rows

    def wait_for_odom(self):
        deadline = rospy.Time.now() + rospy.Duration(20.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            odom, _stamp = self.odom_snapshot()
            if odom is not None:
                return odom
            rate.sleep()
        raise RuntimeError("timed out waiting for odom on {}".format(self.odom_topic))

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    @staticmethod
    def relative_pose(x, y, yaw, x0, y0, yaw0):
        dx = x - x0
        dy = y - y0
        c = math.cos(-yaw0)
        s = math.sin(-yaw0)
        return c * dx - s * dy, s * dx + c * dy, wrap_pi(yaw - yaw0)

    def run(self):
        os.makedirs(self.output_dir, exist_ok=True)
        rows = self.load_dataset()
        rospy.loginfo("loaded %d UGV samples from %s", len(rows), self.dataset_path)

        first_odom = self.wait_for_odom()
        rospy.sleep(self.start_delay_s)

        real0 = rows[0]
        real_x0, real_y0, real_yaw0 = real0[1], real0[2], real0[3]
        sim_x0 = first_odom.pose.pose.position.x
        sim_y0 = first_odom.pose.pose.position.y
        sim_yaw0 = yaw_from_quaternion(first_odom.pose.pose.orientation)
        t0_data = rows[0][0]
        start_ros = rospy.Time.now()

        samples = []
        next_log_time = rospy.Time.now()
        for row in rows:
            if rospy.is_shutdown():
                break
            target_elapsed = max(0.0, row[0] - t0_data)
            while not rospy.is_shutdown():
                now_elapsed = (rospy.Time.now() - start_ros).to_sec()
                if now_elapsed >= target_elapsed:
                    break
                rospy.sleep(min(0.002, target_elapsed - now_elapsed))

            linear = finite(row[self.linear_col]) * self.linear_scale
            angular = finite(row[self.angular_col]) * self.angular_scale
            self.publish_cmd(linear, angular)

            odom, odom_stamp = self.odom_snapshot()
            if odom is None:
                continue
            sim_x = odom.pose.pose.position.x
            sim_y = odom.pose.pose.position.y
            sim_yaw = yaw_from_quaternion(odom.pose.pose.orientation)
            real_x, real_y, real_yaw = row[1], row[2], row[3]

            if self.compare_relative:
                real_cx, real_cy, real_cyaw = self.relative_pose(
                    real_x, real_y, real_yaw, real_x0, real_y0, real_yaw0
                )
                sim_cx, sim_cy, sim_cyaw = self.relative_pose(
                    sim_x, sim_y, sim_yaw, sim_x0, sim_y0, sim_yaw0
                )
            else:
                real_cx, real_cy, real_cyaw = real_x, real_y, real_yaw
                sim_cx, sim_cy, sim_cyaw = sim_x, sim_y, sim_yaw

            pos_error = math.hypot(sim_cx - real_cx, sim_cy - real_cy)
            yaw_error = wrap_pi(sim_cyaw - real_cyaw)
            samples.append(
                {
                    "t": target_elapsed,
                    "real_x": real_cx,
                    "real_y": real_cy,
                    "real_yaw": real_cyaw,
                    "sim_x": sim_cx,
                    "sim_y": sim_cy,
                    "sim_yaw": sim_cyaw,
                    "cmd_linear": linear,
                    "cmd_angular": angular,
                    "sim_linear": odom.twist.twist.linear.x,
                    "sim_angular": odom.twist.twist.angular.z,
                    "pos_error": pos_error,
                    "yaw_error": yaw_error,
                    "odom_age": max(0.0, (rospy.Time.now() - odom_stamp).to_sec()),
                }
            )
            if rospy.Time.now() >= next_log_time:
                rospy.loginfo(
                    "replay t=%.2f pos_err=%.3f yaw_err=%.3f cmd=[%.3f %.3f]",
                    target_elapsed,
                    pos_error,
                    yaw_error,
                    linear,
                    angular,
                )
                next_log_time = rospy.Time.now() + rospy.Duration(2.0)

        self.publish_cmd(0.0, 0.0)
        rospy.sleep(max(0.0, self.stop_after_s))
        self.publish_cmd(0.0, 0.0)

        if not samples:
            raise RuntimeError("no odom samples collected")
        fieldnames = list(samples[0].keys())
        with open(self.csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(samples)

        final = samples[-1]
        pos_errors = [sample["pos_error"] for sample in samples]
        yaw_errors = [sample["yaw_error"] for sample in samples]
        metrics = {
            "dataset_path": self.dataset_path,
            "sample_count": len(samples),
            "duration_s": samples[-1]["t"],
            "cmd_topic": self.cmd_topic,
            "odom_topic": self.odom_topic,
            "linear_col": self.linear_col + 1,
            "angular_col": self.angular_col + 1,
            "linear_scale": self.linear_scale,
            "angular_scale": self.angular_scale,
            "compare_relative": self.compare_relative,
            "position_rmse_m": rmse(pos_errors),
            "position_mean_m": mean(pos_errors),
            "position_p95_m": percentile(pos_errors, 0.95),
            "position_final_m": final["pos_error"],
            "yaw_rmse_rad": rmse(yaw_errors),
            "yaw_mean_abs_rad": mean([abs(value) for value in yaw_errors]),
            "yaw_p95_abs_rad": percentile([abs(value) for value in yaw_errors], 0.95),
            "yaw_final_rad": final["yaw_error"],
            "final_real": {
                "x": final["real_x"],
                "y": final["real_y"],
                "yaw": final["real_yaw"],
            },
            "final_sim": {
                "x": final["sim_x"],
                "y": final["sim_y"],
                "yaw": final["sim_yaw"],
            },
            "csv_path": self.csv_path,
        }
        with open(self.metrics_path, "w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
        rospy.loginfo("wrote replay metrics to %s", self.metrics_path)
        rospy.loginfo(
            "replay result: pos_rmse=%.3f m yaw_rmse=%.3f rad final_pos=%.3f m final_yaw=%.3f rad",
            metrics["position_rmse_m"],
            metrics["yaw_rmse_rad"],
            metrics["position_final_m"],
            metrics["yaw_final_rad"],
        )


def main():
    rospy.init_node("ugv_state_replay_validation")
    node = ReplayValidator()
    node.run()


if __name__ == "__main__":
    main()
