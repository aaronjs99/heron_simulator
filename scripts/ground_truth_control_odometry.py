#!/usr/bin/env python3
"""Publish simulator ground truth as planar body-frame control odometry."""

import copy
import json
import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class GroundTruthControlOdometry:
    def __init__(self):
        rospy.init_node("ground_truth_control_odometry")
        self.input_topic = rospy.get_param("~input_topic", "/pose_gt")
        self.output_topic = rospy.get_param(
            "~output_topic", "/state/sim_control_odometry_3dof"
        )
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=20
        )
        self.status_publisher = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "source": self.input_topic,
                        "output": self.output_topic,
                        "provenance": "simulator_ground_truth",
                        "calibration_eligible": False,
                        "twist_contract": "body_frame_planar",
                    },
                    sort_keys=True,
                )
            )
        )
        rospy.Subscriber(
            self.input_topic, Odometry, self.callback, queue_size=1, tcp_nodelay=True
        )

    def callback(self, source):
        yaw = yaw_from_quaternion(source.pose.pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_vx = float(source.twist.twist.linear.x)
        world_vy = float(source.twist.twist.linear.y)

        output = Odometry()
        output.header.stamp = (
            source.header.stamp
            if source.header.stamp != rospy.Time(0)
            else rospy.Time.now()
        )
        output.header.frame_id = self.parent_frame
        output.child_frame_id = self.child_frame
        output.pose = copy.deepcopy(source.pose)
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)
        output.twist = copy.deepcopy(source.twist)
        output.twist.twist.linear.x = cos_yaw * world_vx + sin_yaw * world_vy
        output.twist.twist.linear.y = -sin_yaw * world_vx + cos_yaw * world_vy
        output.twist.twist.linear.z = 0.0
        output.twist.twist.angular.x = 0.0
        output.twist.twist.angular.y = 0.0
        self.publisher.publish(output)


if __name__ == "__main__":
    GroundTruthControlOdometry()
    rospy.spin()
