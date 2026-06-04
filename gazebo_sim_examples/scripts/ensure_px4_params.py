#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamGet, ParamSet


class Px4ParamEnsurer:
    def __init__(self):
        self.mavros_ns = rospy.get_param("~mavros_ns", "/uav1/mavros").rstrip("/")
        self.wait_timeout_s = float(rospy.get_param("~wait_timeout_s", 20.0))
        self.enable_hover_thrust_estimator = bool(
            rospy.get_param("~enable_hover_thrust_estimator", True)
        )
        self.hover_thrust_estimator_param = rospy.get_param(
            "~hover_thrust_estimator_param",
            "MPC_USE_HTE",
        )
        self.hover_thrust_estimator_value = int(
            rospy.get_param("~hover_thrust_estimator_value", 1)
        )

        self.get_param = rospy.ServiceProxy(self.mavros_ns + "/param/get", ParamGet)
        self.set_param = rospy.ServiceProxy(self.mavros_ns + "/param/set", ParamSet)

    @staticmethod
    def _param_to_value(msg):
        if msg.integer != 0:
            return msg.integer
        return msg.real

    def ensure_int(self, name, expected):
        result = self.get_param(param_id=name)
        if not result.success:
            raise RuntimeError("failed to read PX4 parameter {}".format(name))

        current = self._param_to_value(result.value)
        if int(current) == expected:
            rospy.loginfo("PX4 parameter %s already %s", name, expected)
            return

        value = ParamValue(integer=expected, real=0.0)
        result = self.set_param(param_id=name, value=value)
        if not result.success:
            raise RuntimeError("failed to set PX4 parameter {}={}".format(name, expected))

        applied = self._param_to_value(result.value)
        rospy.loginfo("PX4 parameter %s changed from %s to %s", name, current, applied)

    def run(self):
        if not self.enable_hover_thrust_estimator:
            rospy.loginfo("hover thrust estimator parameter check disabled")
            return 0

        rospy.loginfo(
            "waiting for MAVROS parameter services under %s",
            self.mavros_ns,
        )
        rospy.wait_for_service(self.mavros_ns + "/param/get", timeout=self.wait_timeout_s)
        rospy.wait_for_service(self.mavros_ns + "/param/set", timeout=self.wait_timeout_s)
        self.ensure_int(
            self.hover_thrust_estimator_param,
            self.hover_thrust_estimator_value,
        )
        return 0


def main():
    rospy.init_node("fs150_ensure_px4_params")
    node = Px4ParamEnsurer()
    try:
        raise SystemExit(node.run())
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        rospy.logerr("PX4 parameter ensure failed: %s", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
