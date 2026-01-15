#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf.transformations as tr
import numpy as np


class MapGTPublisher:
    def __init__(self):
        rospy.init_node("map_gt_publisher")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Config
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"

        # Ground truth subscription
        self.gt_sub = rospy.Subscriber("ground_truth/odom", Odometry, self.gt_cb)
        rospy.loginfo("MapGTPublisher: Waiting for messages on ground_truth/odom...")

    def msg_to_matrix(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        trans = [p.x, p.y, p.z]
        rot = [q.x, q.y, q.z, q.w]
        return tr.concatenate_matrices(
            tr.translation_matrix(trans), tr.quaternion_matrix(rot)
        )

    def gt_cb(self, gt_msg):
        rospy.logdebug("Received GT message")
        try:
            # 1. Get T_world_base (Ground Truth)
            t_world_base = self.msg_to_matrix(gt_msg)

            # 2. Get T_odom_base (Current DLIO estimate)
            # lookup_transform returns TransformStamped
            tf_stamped = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0)
            )

            p = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            trans = [p.x, p.y, p.z]
            rot = [q.x, q.y, q.z, q.w]
            t_odom_base = tr.concatenate_matrices(
                tr.translation_matrix(trans), tr.quaternion_matrix(rot)
            )

            # 3. Compute T_map_odom
            # We want: T_map_base == T_world_base
            # T_map_base = T_map_odom * T_odom_base
            # So: T_map_odom = T_world_base * inverse(T_odom_base)

            t_map_odom = np.dot(t_world_base, tr.inverse_matrix(t_odom_base))

            # 4. Publish T_map_odom
            msg = TransformStamped()
            msg.header.stamp = gt_msg.header.stamp
            msg.header.frame_id = self.map_frame
            msg.child_frame_id = self.odom_frame

            trans = tr.translation_from_matrix(t_map_odom)
            rot = tr.quaternion_from_matrix(t_map_odom)

            msg.transform.translation.x = trans[0]
            msg.transform.translation.y = trans[1]
            msg.transform.translation.z = trans[2]
            msg.transform.rotation.x = rot[0]
            msg.transform.rotation.y = rot[1]
            msg.transform.rotation.z = rot[2]
            msg.transform.rotation.w = rot[3]

            self.br.sendTransform(msg)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass


if __name__ == "__main__":
    try:
        node = MapGTPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
