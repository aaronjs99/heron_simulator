#!/usr/bin/env python3
# Software License Agreement (BSD)

from geometry_msgs.msg import Wrench
from heron_msgs.msg import Drive
import rospy


class ThrusterTranslator:
    """Translate normalized Drive commands to thruster wrench inputs.

    Active sim path:
      /cmd_drive -> /thrusters/{1,0}/input

    This node mirrors the HERON controller-level thrust model:
    - clamp cmd_drive to [-1, 1]
    - optional per-side scaling
    - piecewise linear thrust mapping:
        cmd >= 0: thrust = cmd * max_fwd_thrust
        cmd < 0 : thrust = cmd * max_bck_thrust
    - timeout to zero
    It should not invent actuator behavior not present in repo-truth controller
    logic; novelty belongs upstream from /cmd_drive.
    """

    def __init__(self):
        rospy.init_node("cmd_drive_to_thrusters")

        namespace = rospy.get_param("~namespace", "")

        prefix = f"/{namespace}" if namespace else ""

        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.75))
        self.max_fwd_thrust = float(rospy.get_param("~max_fwd_thrust", 45.0))
        self.max_bck_thrust = float(rospy.get_param("~max_bck_thrust", 25.0))
        self.left_scale = float(rospy.get_param("~left_scale", 1.0))
        self.right_scale = float(rospy.get_param("~right_scale", 1.0))

        default_left_topic = (
            f"{prefix}/thrusters/1/input" if prefix else "/thrusters/1/input"
        )
        default_right_topic = (
            f"{prefix}/thrusters/0/input" if prefix else "/thrusters/0/input"
        )
        default_drive_topic = "cmd_drive"

        left_topic = rospy.get_param("~left_thruster_topic", default_left_topic)
        right_topic = rospy.get_param("~right_thruster_topic", default_right_topic)
        drive_topic = rospy.get_param("~drive_topic", default_drive_topic)

        self.p_left = rospy.Publisher(left_topic, Wrench, queue_size=1)
        self.p_right = rospy.Publisher(right_topic, Wrench, queue_size=1)

        self.sub = rospy.Subscriber(drive_topic, Drive, self.callback)
        self.target_left = 0.0
        self.target_right = 0.0
        self.last_cmd_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.update)
        rospy.loginfo(
            "Thruster translator initialized for namespace: %s drive=%s left=%s right=%s",
            namespace,
            drive_topic,
            left_topic,
            right_topic,
        )

    def callback(self, msg):
        self.target_left = self.shape_drive(msg.left, self.left_scale)
        self.target_right = self.shape_drive(msg.right, self.right_scale)
        self.last_cmd_time = rospy.Time.now()

    def shape_drive(self, cmd, scale):
        cmd = max(-1.0, min(1.0, float(cmd)))
        return max(-1.0, min(1.0, cmd * float(scale)))

    def drive_to_thrust(self, drive):
        if drive >= 0.0:
            return drive * self.max_fwd_thrust
        return drive * self.max_bck_thrust

    def update(self, _event):
        now = rospy.Time.now()
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.target_left = 0.0
            self.target_right = 0.0

        left_wrench = Wrench()
        left_wrench.force.x = self.drive_to_thrust(self.target_left)
        self.p_left.publish(left_wrench)

        right_wrench = Wrench()
        right_wrench.force.x = self.drive_to_thrust(self.target_right)
        self.p_right.publish(right_wrench)


if __name__ == "__main__":
    try:
        ThrusterTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
