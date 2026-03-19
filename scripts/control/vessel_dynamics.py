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
        self.mass = 38.0
        self.volume = 0.1  # Realistic displacement volume for catamaran hulls
        self.rho = 1025.0  # Seawater density (approx)
        self.g = 9.81
        self.hull_height = 0.32
        self.water_level = 0.0  # Ocean visual is at z=0

        # --- Added Mass Matrix (M_a) ---
        # Tuned for high payload (38kg)
        self.Ma = np.diag([30.0, 60.0, 60.0, 0.0, 30.0, 30.0])

        # --- Damping Matrices ---
        # Scaled for 38kg to maintain 'heavy' realistic feel
        self.D_linear = np.diag([30.0, 70.0, 90.0, 20.0, 45.0, 45.0])
        self.D_quad = np.diag([15.0, 45.0, 80.0, 15.0, 45.0, 45.0])

        # --- Stability ---
        self.gm_roll = 0.2
        self.gm_pitch = 0.2
        self.rho_air = 1.225
        # The Heron has low freeboard, so calm-water wind loads should stay modest by default.
        self.wind_drag_coeff = float(rospy.get_param("~wind_drag_coeff", 0.85))
        self.wind_area = np.array(
            rospy.get_param("~wind_area", [0.18, 0.45]), dtype=float
        )

        self.current_mean = np.array(
            rospy.get_param("~water_current_mean", [0.08, 0.02, 0.0]), dtype=float
        )
        self.current_std = np.array(
            rospy.get_param("~water_current_std", [0.03, 0.02, 0.0]), dtype=float
        )
        self.current_tau = float(rospy.get_param("~water_current_tau", 15.0))

        self.wind_mean = np.array(
            rospy.get_param("~wind_mean", [0.25, 0.08, 0.0]), dtype=float
        )
        self.wind_std = np.array(
            rospy.get_param("~wind_std", [0.12, 0.08, 0.0]), dtype=float
        )
        self.wind_tau = float(rospy.get_param("~wind_tau", 16.0))
        self.wind_speed_limit = float(rospy.get_param("~wind_speed_limit", 0.6))

        self.wave_force_std = np.array(
            rospy.get_param("~wave_force_std", [0.0, 1.5, 4.0]), dtype=float
        )
        self.wave_torque_std = np.array(
            rospy.get_param("~wave_torque_std", [1.2, 1.2, 0.25]), dtype=float
        )
        self.wave_tau = float(rospy.get_param("~wave_tau", 2.5))
        self.rng = np.random.default_rng(
            int(rospy.get_param("~random_seed", rospy.Time.now().to_nsec() % 2**32))
        )
        self.current_state = self.current_mean.copy()
        self.wind_state = self.wind_mean.copy()
        self.wave_force_state = np.zeros(3)
        self.wave_torque_state = np.zeros(3)
        self.last_stamp = None

        # --- ROS Setup ---
        namespace = rospy.get_param("~namespace", "")
        odom_topic = rospy.get_param("~odom_topic", "pose_gt")
        topic_name = f"/{namespace}/hydro_forces" if namespace else "/hydro_forces"
        self.pub = rospy.Publisher(topic_name, Wrench, queue_size=1)
        self.sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb)

        rospy.loginfo("Fossen Dynamics Engine initialized (Heavy Tune).")

    def update_ou_state(self, state, mean, std, tau, dt):
        """Advance an Ornstein-Uhlenbeck disturbance state."""
        if tau <= 1e-6:
            return mean.copy()
        sigma = np.sqrt(2.0 / tau) * std
        noise = self.rng.normal(0.0, 1.0, size=state.shape)
        return state + ((mean - state) / tau) * dt + sigma * np.sqrt(dt) * noise

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
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_stamp is None:
            dt = 0.02
        else:
            dt = max((stamp - self.last_stamp).to_sec(), 1e-3)
        self.last_stamp = stamp

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

        self.current_state = self.update_ou_state(
            self.current_state, self.current_mean, self.current_std, self.current_tau, dt
        )
        self.wind_state = self.update_ou_state(
            self.wind_state, self.wind_mean, self.wind_std, self.wind_tau, dt
        )
        wind_xy = self.wind_state[:2]
        wind_xy_speed = np.linalg.norm(wind_xy)
        if self.wind_speed_limit > 1e-6 and wind_xy_speed > self.wind_speed_limit:
            self.wind_state[:2] = wind_xy * (self.wind_speed_limit / wind_xy_speed)
        self.wave_force_state = self.update_ou_state(
            self.wave_force_state, np.zeros(3), self.wave_force_std, self.wave_tau, dt
        )
        self.wave_torque_state = self.update_ou_state(
            self.wave_torque_state, np.zeros(3), self.wave_torque_std, self.wave_tau, dt
        )

        current_body = np.dot(rot_mat.T, self.current_state)
        wind_body = np.dot(rot_mat.T, self.wind_state)
        rel_water_lin = nu_lin - current_body
        rel_air_lin = wind_body - nu_lin

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
        rel_nu = np.concatenate([rel_water_lin, nu_ang])
        D_total = self.D_linear + np.diag(np.diag(self.D_quad) * np.abs(rel_nu))
        f_damping = -np.dot(D_total, rel_nu)

        # 4. Coriolis Forces C(nu)*nu
        # Only implementing Rigid Body Coriolis (C_rb) for simplicity,
        # acts to destabilize/couple axes during turns.
        f_coriolis_lin = -self.mass * np.cross(nu_ang, rel_water_lin)
        f_coriolis = np.concatenate([f_coriolis_lin, [0, 0, 0]])

        wind_force = np.array(
            [
                0.5
                * self.rho_air
                * self.wind_drag_coeff
                * self.wind_area[0]
                * rel_air_lin[0]
                * abs(rel_air_lin[0]),
                0.5
                * self.rho_air
                * self.wind_drag_coeff
                * self.wind_area[1]
                * rel_air_lin[1]
                * abs(rel_air_lin[1]),
                0.0,
            ]
        )
        wind_torque = np.array(
            [
                0.10 * wind_force[1],
                -0.08 * wind_force[0],
                0.20 * wind_force[1],
            ]
        )

        # 5. Summation
        wrench = Wrench()
        # Linear Forces
        wrench.force.x = (
            f_damping[0] + f_coriolis[0] + f_b_body[0] + wind_force[0] + self.wave_force_state[0]
        )
        wrench.force.y = (
            f_damping[1] + f_coriolis[1] + f_b_body[1] + wind_force[1] + self.wave_force_state[1]
        )
        wrench.force.z = f_damping[2] + f_coriolis[2] + f_b_body[2] + self.wave_force_state[2]

        # Torques
        wrench.torque.x = (
            f_damping[3] + f_coriolis[3] + tau_restoring[0] + wind_torque[0] + self.wave_torque_state[0]
        )
        wrench.torque.y = (
            f_damping[4] + f_coriolis[4] + tau_restoring[1] + wind_torque[1] + self.wave_torque_state[1]
        )
        wrench.torque.z = (
            f_damping[5] + f_coriolis[5] + tau_restoring[2] + wind_torque[2] + self.wave_torque_state[2]
        )

        self.pub.publish(wrench)


if __name__ == "__main__":
    try:
        VesselDynamics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
