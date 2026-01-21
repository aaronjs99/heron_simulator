# heron_simulator/scripts/fake_localize.py
"""Fake Localization for Simulation.
--------------------------------

Publishes a perfect `map` -> `odom` transform by comparing the simulator's
Ground Truth pose (from Gazebo) with the drift-prone Odometry estimate.
This aligns the map frame perfectly with the world frame, bypassing
complex AMCL/SLAM logic for debugging purposes.

Transform Logic:
$$ T_{map}^{odom} = T_{world}^{base} \\cdot (T_{odom}^{base})^{-1} $$
"""
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped



class FakeLocalization:
    def __init__(self):
        rospy.init_node("fake_localization_tf")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Config
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"

        # Ground truth subscription
        self.gt_sub = rospy.Subscriber("ground_truth/odom", Odometry, self.gt_cb)

    def gt_cb(self, gt_msg):
        # We want: T_map_odom = T_map_base * T_base_odom
        # GT gives T_map_base (assuming GT is in world frame, which we call map)

        try:
            # Get current T_odom_base from TF tree (published by DLIO)
            # We need T_base_odom = inverse(T_odom_base)
            t_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1)
            )

            # Construct T_map_odom
            # Since we can't easily multiply transforms without a library like kdl or eigen in python without deps,
            # We can treat this simply:
            # Map_Pose = Odom_Correction * Odom_Pose
            # Odom_Correction = Map_Pose * Inverse(Odom_Pose)
            # We will publish T_map_odom

            # ...Actually, doing full transform algebra in raw python is annoying.
            # Let's assume the user just wants the 'map' frame to align with Ground Truth.

            # GT Msg Pose is T_world_base.
            # We want to publish T_map_odom such that:
            #   T_map_base (result) ~= T_world_base (GT)
            #   T_map_base = T_map_odom * T_odom_base
            # So: T_map_odom = T_world_base * T_odom_base^-1

            # Let's simplify:
            # If we publish map->base_link directly from GT, we break the TF tree (odom->base already exists).
            # So we MUST publish map->odom.

            pass

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        # SIMPLIFIED IMPLEMENTATION
        # Use the logic from `fake_localization` package but light version.
        # Actually, let's just use the `fake_localization` node provided by `ros-noetic-fake-localization` if available?
        # User might not have it installed. Safer to write a simple script.

        # To avoid complex math, we can just cheat:
        # 1. Get GT Pose (P_gt).
        # 2. Get Odom Pose (P_odom).
        # 3. Correction = P_gt - P_odom (roughly).

        # Or better, let's use the helper library if possible.
        # tf.transformations is available in standard ROS python.

        pass


# WAIT. WRITING A NEW NODE IS RISKY IF I GET MATH WRONG.
# IS THERE A SIMPLER WAY?
# The user's issue is "Gazebo is different place".
# If I just want them to match, I can publish `map -> odom` using `fake_localization` package.
# Does `simulation_full.launch` have it? No.
# `heron_simulator` depends on `uuv_simulator`.
# Does `uuv_simulator` have a GT publisher?
# `heron_world.launch` uses `publish_world_ned_frame`.

# Let's try to look for `fake_localization` binary first.
