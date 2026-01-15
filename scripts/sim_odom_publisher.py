#!/usr/bin/env python3
"""
Simulation odometry publisher.
Subscribes to ground_truth/odom and republishes TF + odom for navigation stack.
"""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class SimOdomPublisher:
    def __init__(self):
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", False)  # planar_move does this

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        rospy.Subscriber("/ground_truth/odom", Odometry, self.odom_callback)
        rospy.loginfo(
            "Sim odom publisher: ground_truth/odom -> /odom (TF: %s)",
            "enabled" if self.publish_tf else "disabled (using planar_move TF)",
        )

    def odom_callback(self, msg: Odometry):
        # Republish with correct frame IDs
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

        # Optionally publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node("sim_odom_publisher")
    SimOdomPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
