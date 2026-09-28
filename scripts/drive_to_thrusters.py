#!/usr/bin/env python3
# Software License Agreement (BSD)

"""Bridge normalized Heron drive commands into Gazebo thruster wrench inputs."""

import json

from geometry_msgs.msg import Wrench
from heron_msgs.msg import Drive
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

from models.four_regime_propulsion import propulsion_output


def clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def slew_toward(current, target, max_delta):
    if max_delta <= 0.0:
        return target
    return current + clamp(target - current, -max_delta, max_delta)


class DriveToThrusters(Node):
    """Translate normalized Drive commands to thruster wrench inputs.

    Active sim path:
      /cmd_drive -> /thrusters/{left,right}/input

    The default simulation plant applies four independent static propulsion
    regimes (left/right by forward/reverse):
    - clamp cmd_drive to [-1, 1]
    - optional per-side scaling
    - optional first-order actuator lag and drive-space slew limiting
    - regime-specific deadband and nonlinear force exponent
    - direction-specific maximum force with squared voltage scaling
    - synthetic current/RPM telemetry that is never calibration evidence
    - timeout to zero
    It should not invent actuator behavior not present in repo-truth controller
    logic; novelty belongs upstream from /cmd_drive.
    """

    def __init__(self):
        super().__init__("cmd_drive_to_thrusters")

        namespace = self._parameter("namespace", "")

        prefix = f"/{namespace}" if namespace else ""

        self.rate_hz = float(self._parameter("rate", 30.0))
        if self.rate_hz <= 0.0:
            raise ValueError("rate must be positive")
        self.cmd_timeout = float(self._parameter("cmd_timeout", 0.75))
        self.max_fwd_thrust = float(self._parameter("max_fwd_thrust", 45.0))
        self.max_bck_thrust = float(self._parameter("max_bck_thrust", 25.0))
        self.left_scale = float(self._parameter("left_scale", 1.0))
        self.right_scale = float(self._parameter("right_scale", 1.0))
        self.response_time_constant_sec = max(
            0.0, float(self._parameter("response_time_constant_sec", 0.0))
        )
        self.max_drive_delta_per_sec = max(
            0.0, float(self._parameter("max_drive_delta_per_sec", 0.0))
        )
        self.nominal_voltage_v = max(
            1e-6, float(self._parameter("nominal_voltage_v", 16.0))
        )
        self.simulated_voltage_v = max(
            1e-6, float(self._parameter("simulated_voltage_v", 16.0))
        )
        self.direction_change_blank_sec = max(
            0.0, float(self._parameter("direction_change_blank_sec", 0.0))
        )
        self.regimes = {}
        for side in ("left", "right"):
            for direction in ("forward", "reverse"):
                key = "{}_{}".format(side, direction)
                self.regimes[key] = {
                    "deadband": float(
                        self._parameter("regimes.{}.deadband".format(key), 0.1)
                    ),
                    "force_exponent": float(
                        self._parameter("regimes.{}.force_exponent".format(key), 1.25)
                    ),
                    "max_current_a": float(
                        self._parameter(
                            "regimes.{}.max_current_a".format(key),
                            6.0 if direction == "forward" else 1.2,
                        ).value
                    ),
                    "max_force_n": float(
                        self._parameter(
                            "regimes.{}.max_force_n".format(key),
                            (
                                self.max_fwd_thrust
                                if direction == "forward"
                                else self.max_bck_thrust
                            ),
                        ).value
                    ),
                    "nominal_voltage_v": float(
                        self._parameter(
                            "regimes.{}.nominal_voltage_v".format(key),
                            self.nominal_voltage_v,
                        ).value
                    ),
                    "voltage_exponent": float(
                        self._parameter("regimes.{}.voltage_exponent".format(key), 2.0)
                    ),
                    "max_rpm": float(
                        self._parameter("regimes.{}.max_rpm".format(key), 5500.0)
                    ),
                }
        self.synthetic_current_a = {"left": 0.0, "right": 0.0}
        self.synthetic_rpm = {"left": 0.0, "right": 0.0}
        self.synthetic_pwm_us = {"left": 1500.0, "right": 1500.0}
        self.direction_sign = {"left": 0, "right": 0}
        zero_time = self.get_clock().now()
        self.direction_blank_until = {
            "left": zero_time,
            "right": zero_time,
        }

        default_left_topic = (
            f"{prefix}/thrusters/left/input" if prefix else "/thrusters/left/input"
        )
        default_right_topic = (
            f"{prefix}/thrusters/right/input" if prefix else "/thrusters/right/input"
        )
        default_drive_topic = "cmd_drive"

        left_topic = self._parameter("left_thruster_topic", default_left_topic)
        right_topic = self._parameter("right_thruster_topic", default_right_topic)
        drive_topic = self._parameter("drive_topic", default_drive_topic)

        self.p_left = self.create_publisher(Wrench, left_topic, 1)
        self.p_right = self.create_publisher(Wrench, right_topic, 1)
        self.actuator_state_pub = self.create_publisher(String, "~/actuator_state", 10)

        self.sub = self.create_subscription(Drive, drive_topic, self.callback, 10)
        self.target_left = 0.0
        self.target_right = 0.0
        self.actual_left = 0.0
        self.actual_right = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.update)
        self.get_logger().info(
            "Drive-to-thrusters bridge initialized: "
            f"namespace={namespace} drive={drive_topic} left={left_topic} "
            f"right={right_topic} left_scale={self.left_scale:.3f} "
            f"right_scale={self.right_scale:.3f} "
            f"tau={self.response_time_constant_sec:.3f}s "
            f"max_delta={self.max_drive_delta_per_sec:.3f}/s"
        )

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def callback(self, msg):
        self.target_left = self.shape_drive(msg.left, self.left_scale)
        self.target_right = self.shape_drive(msg.right, self.right_scale)
        self.last_cmd_time = self.get_clock().now()

    def shape_drive(self, cmd, scale):
        cmd = clamp(float(cmd), -1.0, 1.0)
        return clamp(cmd * float(scale), -1.0, 1.0)

    def drive_to_thrust(self, drive, side):
        if abs(drive) <= 1e-12:
            self.synthetic_current_a[side] = 0.0
            self.synthetic_rpm[side] = 0.0
            self.synthetic_pwm_us[side] = 1500.0
            return 0.0
        direction = "forward" if drive >= 0.0 else "reverse"
        regime = self.regimes["{}_{}".format(side, direction)]
        values = propulsion_output(drive, regime, self.simulated_voltage_v)
        self.synthetic_current_a[side] = values["current_a"]
        self.synthetic_rpm[side] = values["rpm"]
        self.synthetic_pwm_us[side] = values["pwm_us"]
        return values["force_n"]

    def reset_actuator_epoch(self, now):
        """Clear all command, lag, and telemetry state on time reset."""
        self.target_left = 0.0
        self.target_right = 0.0
        self.actual_left = 0.0
        self.actual_right = 0.0
        self.synthetic_current_a = {"left": 0.0, "right": 0.0}
        self.synthetic_rpm = {"left": 0.0, "right": 0.0}
        self.synthetic_pwm_us = {"left": 1500.0, "right": 1500.0}
        self.direction_sign = {"left": 0, "right": 0}
        self.direction_blank_until = {"left": now, "right": now}
        self.last_cmd_time = now
        self.last_update_time = now

    def update(self):
        now = self.get_clock().now()
        if now < self.last_update_time or now < self.last_cmd_time:
            self.get_logger().warning(
                "Simulation time rolled back; clearing thruster state"
            )
            self.reset_actuator_epoch(now)
        dt = max(0.0, (now - self.last_update_time).nanoseconds / 1e9)
        self.last_update_time = now
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.target_left = 0.0
            self.target_right = 0.0

        target_left = self.target_left
        target_right = self.target_right
        if self.response_time_constant_sec > 1e-6 and dt > 0.0:
            alpha = min(1.0, dt / self.response_time_constant_sec)
            target_left = self.actual_left + (
                (self.target_left - self.actual_left) * alpha
            )
            target_right = self.actual_right + (
                (self.target_right - self.actual_right) * alpha
            )
        max_delta = self.max_drive_delta_per_sec * dt
        self.actual_left = slew_toward(self.actual_left, target_left, max_delta)
        self.actual_right = slew_toward(self.actual_right, target_right, max_delta)
        # No measured propeller-inertia model exists. Preserve the safety
        # contract that explicit zero or command timeout means zero commanded
        # wrench, rather than inventing a residual thrust tail from drive lag.
        if abs(self.target_left) <= 1e-12:
            self.actual_left = 0.0
        if abs(self.target_right) <= 1e-12:
            self.actual_right = 0.0

        for side, drive in (
            ("left", self.actual_left),
            ("right", self.actual_right),
        ):
            sign = 1 if drive > 0.0 else -1 if drive < 0.0 else 0
            if sign and self.direction_sign[side] and sign != self.direction_sign[side]:
                self.direction_blank_until[side] = now + Duration(
                    seconds=self.direction_change_blank_sec
                )
            if sign:
                self.direction_sign[side] = sign

        left_wrench = Wrench()
        if now < self.direction_blank_until["left"]:
            left_wrench.force.x = 0.0
            self.synthetic_current_a["left"] = 0.0
            self.synthetic_rpm["left"] = 0.0
            self.synthetic_pwm_us["left"] = 1500.0
        else:
            left_wrench.force.x = self.drive_to_thrust(self.actual_left, "left")
        self.p_left.publish(left_wrench)

        right_wrench = Wrench()
        if now < self.direction_blank_until["right"]:
            right_wrench.force.x = 0.0
            self.synthetic_current_a["right"] = 0.0
            self.synthetic_rpm["right"] = 0.0
            self.synthetic_pwm_us["right"] = 1500.0
        else:
            right_wrench.force.x = self.drive_to_thrust(self.actual_right, "right")
        self.p_right.publish(right_wrench)
        self.actuator_state_pub.publish(
            String(
                data=json.dumps(
                    {
                        "source": "synthetic_simulation",
                        "calibration_eligible": False,
                        "physical_telemetry": False,
                        "model_version": "provisional-four-regime-v1",
                        "evidence_status": "provisional_simulation_only",
                        "current_semantics": "synthetic_battery_side_current",
                        "left_current_a": self.synthetic_current_a["left"],
                        "right_current_a": self.synthetic_current_a["right"],
                        "left_rpm": self.synthetic_rpm["left"],
                        "right_rpm": self.synthetic_rpm["right"],
                        "left_pwm_us": self.synthetic_pwm_us["left"],
                        "right_pwm_us": self.synthetic_pwm_us["right"],
                        "battery_voltage_v": self.simulated_voltage_v,
                        "left_drive": self.actual_left,
                        "right_drive": self.actual_right,
                        "left_force_proxy_n": left_wrench.force.x,
                        "right_force_proxy_n": right_wrench.force.x,
                    },
                    sort_keys=True,
                    allow_nan=False,
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveToThrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
