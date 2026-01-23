#!/usr/bin/env python3
"""
Vessel Dynamics Engine for Heron USV.
------------------------------------

Implements a high-fidelity hydrodynamic model using the Fossen equations 
(linear/quadratic damping and hydrostatics) to replace the UUV Simulator 
plugins. This node calculates forces/torques based on the vessel's state 
and applies them to the Gazebo physics engine via the 'hydro_forces' topic.

Model Equations:
  F_hydro = F_buoyancy + F_damping + F_restorative

Where:
  - F_buoyancy: Vertical force based on submerged volume.
  - F_damping: Viscous drag proportional to linear and quadratic velocity.
  - F_restorative: Torques aligning the vessel to the water surface.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Wrench, PoseStamped, Vector3
from nav_msgs.msg import Odometry
import tf.transformations as tr


class VesselDynamics:
    def __init__(self):
        rospy.init_node("vessel_dynamics")

        # Configuration (Values from gazebo.xacro and empirical trials)
        self.mass = 28.0
        self.volume = 0.13
        self.rho = 1000.0  # Density of water (kg/m^3)
        self.gravity = 9.81
        self.hull_height = 0.32
        self.water_level = 0.0

        # Damping Coefficients (X, Y, Z, K, M, N)
        self.linear_damping = np.array([25.0, 24.0, 150.0, 20.0, 20.0, 10.0])
        self.quadratic_damping = np.array([5.0, 5.0, 15.0, 8.0, 8.0, 8.0])

        # Metacentric stability (Restorative moments)
        self.gm_roll = 0.1  # Metacentric height for roll (m)
        self.gm_pitch = 0.1  # Metacentric height for pitch (m)

        # Publishers / Subscribers
        namespace = rospy.get_param("~namespace", "heron")
        topic_name = f"/{namespace}/hydro_forces" if namespace else "/hydro_forces"
        self.pub = rospy.Publisher(topic_name, Wrench, queue_size=1)
        self.sub = rospy.Subscriber("ground_truth/odom", Odometry, self.odom_cb)

        rospy.loginfo("Vessel Dynamics Engine initialized.")

    def odom_cb(self, msg):
        """Calculate and publish hydrodynamic forces based on current state."""

        # 1. Extract State
        p = msg.pose.pose.position
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        # Velocities are in body frame in Gazebo Odom usually, but let's be sure.
        # Odometry message usually has twist in child_frame_id (base_link).
        v_lin = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        v_ang = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )

        # 2. Calculate Damping (in body frame)
        # Force = -(L*v + Q*v*|v|)
        v_all = np.concatenate([v_lin, v_ang])
        f_damping_all = -(
            self.linear_damping * v_all + self.quadratic_damping * v_all * np.abs(v_all)
        )

        # 3. Calculate Buoyancy (in world frame, then transform to body)
        # Simple vertical force: F_b = rho * g * V_submerged
        # Submerged fraction: clamp((water_level - p.z) / hull_height + 0.5, 0, 1)
        # (Offset by 0.5 assuming COG is at center of hull)
        draft = self.water_level - p.z
        sub_ratio = np.clip((draft / self.hull_height) + 0.5, 0.0, 1.0)
        f_buoyancy_val = self.rho * self.gravity * self.volume * sub_ratio

        # World frame buoyancy vector
        f_b_world = np.array([0, 0, f_buoyancy_val])

        # Rotate buoyancy to body frame
        rot_mat = tr.quaternion_matrix(q)[:3, :3]
        f_b_body = np.dot(rot_mat.T, f_b_world)

        # 4. Restorative Moments (Stability)
        # Using simplified metacentric formula: Torque = -mass * g * GM * sin(theta)
        r, p_angle, y = tr.euler_from_quaternion(q)
        tau_roll = -self.mass * self.gravity * self.gm_roll * np.sin(r)
        tau_pitch = -self.mass * self.gravity * self.gm_pitch * np.sin(p_angle)

        # 5. Combine and Publish
        wrench = Wrench()
        wrench.force.x = f_damping_all[0] + f_b_body[0]
        wrench.force.y = f_damping_all[1] + f_b_body[1]
        wrench.force.z = f_damping_all[2] + f_b_body[2]

        wrench.torque.x = f_damping_all[3] + tau_roll
        wrench.torque.y = f_damping_all[4] + tau_pitch
        wrench.torque.z = f_damping_all[5]

        self.pub.publish(wrench)


if __name__ == "__main__":
    try:
        engine = VesselDynamics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
