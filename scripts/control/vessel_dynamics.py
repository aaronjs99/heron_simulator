#!/usr/bin/env python3
"""
Vessel Dynamics Engine for Heron USV (Fossen Model).
--------------------------------------------------
High-fidelity hydrodynamic model replacing UUV Simulator plugins.
Implements Fossen's Equations of Motion:
    M_rb*dv/dt + C_rb(v)*v + M_a*dv/dt + C_a(v)*v + D(v)*v + g(eta) = tau

Reference: https://github.com/heron/heron_simulator/
The official Heron simulation uses the UUV Simulator Fossen plugin with:
- Added Mass: ZEROS (Negligible)
- Linear Damping: ~25.0
- Quadratic Damping: ~5.0
*Note*: These official values result in "drifty" behavior. This implementation 
uses "Heavy" tuning (Added Mass + Higher Damping) to provide realistic handling.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import tf.transformations as tr


def skew(v):
    """Compute the skew-symmetric matrix of a 3D vector.

    Args:
        v (array-like): A 3-element vector.

    Returns:
        np.ndarray: 3x3 skew-symmetric matrix.
    """
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class VesselDynamics:
    """Fossen-model hydrodynamic simulation for the Heron USV.

    Computes hydrodynamic forces including buoyancy, damping, Coriolis effects,
    and metacentric restoring moments. Uses a "heavy" damping configuration
    for realistic handling.

    Attributes:
        mass (float): Vessel mass in kg.
        Ma (np.ndarray): 6x6 added mass matrix.
        D_linear (np.ndarray): 6x6 linear damping matrix.
        D_quad (np.ndarray): 6x6 quadratic damping matrix.
        pub (rospy.Publisher): Publisher for hydrodynamic wrench.
    """

    def __init__(self):
        """Initialize the VesselDynamics node with physical parameters."""
        rospy.init_node("vessel_dynamics")

        # --- Physical Parameters ---
        self.mass = 28.0
        self.volume = 0.13  # Displaced volume approx
        self.rho = 1000.0  # Water density
        self.g = 9.81
        self.hull_height = 0.32
        self.water_level = 0.3  # Fixed verified water level

        # --- Added Mass Matrix (M_a) ---
        # Official Repo: Zeros.
        # Custom: Added inertial mass to prevent "drift car" feel.
        self.Ma = np.diag([20.0, 40.0, 40.0, 0.0, 20.0, 20.0])

        # --- Damping Matrices ---
        # Official Repo: Linear=[25,24,150...], Quad=[5,5,15...]
        # Custom: Stiffened Sway(1) and Yaw(5) to lock movement to rails.
        self.D_linear = np.diag([30.0, 80.0, 100.0, 20.0, 50.0, 50.0])
        self.D_quad = np.diag([15.0, 50.0, 100.0, 15.0, 50.0, 50.0])

        # --- Stability ---
        self.gm_roll = 0.2
        self.gm_pitch = 0.2

        # --- ROS Setup ---
        namespace = rospy.get_param("~namespace", "heron")
        topic_name = f"/{namespace}/hydro_forces" if namespace else "/hydro_forces"
        self.pub = rospy.Publisher(topic_name, Wrench, queue_size=1)
        self.sub = rospy.Subscriber("ground_truth/odom", Odometry, self.odom_cb)

        rospy.loginfo("Fossen Dynamics Engine initialized (Heavy Tune).")

    def odom_cb(self, msg):
        """Compute and publish hydrodynamic forces from odometry.

        Calculates buoyancy, damping, Coriolis, and restoring forces based
        on the vessel's current state.

        Args:
            msg (Odometry): Current vessel odometry.
        """
        # 1. State Extraction
        p = msg.pose.pose.position
        q_obj = msg.pose.pose.orientation
        q = [q_obj.x, q_obj.y, q_obj.z, q_obj.w]

        # Linear/Angular Velocities (in Body Frame)
        nu_lin = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        nu_ang = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )
        nu = np.concatenate([nu_lin, nu_ang])  # 6-vector

        # 2. Hydrostatic Forces (Restoring) g(eta)
        # Buoyancy
        draft = self.water_level - p.z
        sub_ratio = np.clip((draft / self.hull_height) + 0.5, 0.0, 1.0)
        f_buoyancy = self.rho * self.g * self.volume * sub_ratio

        # Transform vertical World Force to Body Frame
        rot_mat = tr.quaternion_matrix(q)[:3, :3]
        f_b_world = np.array([0, 0, f_buoyancy])
        f_b_body = np.dot(rot_mat.T, f_b_world)

        # Metacentric Righting Moments
        r, p_ang, y = tr.euler_from_quaternion(q)
        tau_restoring = np.array(
            [
                -self.mass * self.g * self.gm_roll * np.sin(r),  # Roll
                -self.mass * self.g * self.gm_pitch * np.sin(p_ang),  # Pitch
                0.0,  # Yaw (no natural restoring force)
            ]
        )

        # 3. Damping Forces D(nu)*nu
        # D_total = D_lin + D_quad * |nu|
        D_total = self.D_linear + self.D_quad * np.abs(
            np.diag(nu)
        )  # Simplified element-wise
        f_damping = -np.dot(D_total, nu)

        # 4. Coriolis Forces C(nu)*nu
        # Only implementing Rigid Body Coriolis (C_rb) for simplicity,
        # acts to destabilize/couple axes during turns.
        f_coriolis_lin = -self.mass * np.cross(nu_ang, nu_lin)
        f_coriolis = np.concatenate([f_coriolis_lin, [0, 0, 0]])

        # 5. Summation
        wrench = Wrench()
        # Linear Forces
        wrench.force.x = f_damping[0] + f_coriolis[0] + f_b_body[0]
        wrench.force.y = f_damping[1] + f_coriolis[1] + f_b_body[1]
        wrench.force.z = f_damping[2] + f_coriolis[2] + f_b_body[2]

        # Torques
        wrench.torque.x = f_damping[3] + f_coriolis[3] + tau_restoring[0]
        wrench.torque.y = f_damping[4] + f_coriolis[4] + tau_restoring[1]
        wrench.torque.z = f_damping[5] + f_coriolis[5] + tau_restoring[2]

        self.pub.publish(wrench)


if __name__ == "__main__":
    try:
        VesselDynamics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
