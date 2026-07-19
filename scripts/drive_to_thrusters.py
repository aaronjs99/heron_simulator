#!/usr/bin/env python3
# Software License Agreement (BSD)

"""Bridge normalized Heron drive commands into Gazebo thruster wrench inputs."""

import json
from pathlib import Path
import sys

from geometry_msgs.msg import Wrench
from heron_msgs.msg import Drive
import rospy
import rospkg
from std_msgs.msg import String

# Catkin wraps executable scripts in the devel space. Load the pure helper from
# the package source directory so validators and runtime share one contract.
sys.path.insert(
    0, str(Path(rospkg.RosPack().get_path("heron_simulator")) / "scripts")
)
from empirical_actuator_proxy import (
    curve_effort,
    payload_sha256,
    select_hysteresis_sweep,
    validate_proxy,
)


def clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def slew_toward(current, target, max_delta):
    if max_delta <= 0.0:
        return target
    return current + clamp(target - current, -max_delta, max_delta)


class DriveToThrusters:
    """Translate normalized Drive commands to thruster wrench inputs.

    Active sim path:
      /cmd_drive -> /thrusters/{1,0}/input

    This node mirrors the HERON controller-level thrust model:
    - clamp cmd_drive to [-1, 1]
    - optional per-side scaling
    - optional first-order actuator lag and drive-space slew limiting
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
        self.response_time_constant_sec = max(
            0.0, float(rospy.get_param("~response_time_constant_sec", 0.0))
        )
        self.max_drive_delta_per_sec = max(
            0.0, float(rospy.get_param("~max_drive_delta_per_sec", 0.0))
        )
        self.empirical_model_enabled = bool(
            rospy.get_param("~empirical_model_enabled", False)
        )
        self.empirical_model_file = str(
            rospy.get_param("~empirical_model_file", "")
        )
        self.empirical_model = self.load_empirical_model()

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
        self.model_status_pub = rospy.Publisher(
            "~actuator_proxy_status", String, queue_size=1, latch=True
        )

        self.sub = rospy.Subscriber(drive_topic, Drive, self.callback)
        self.target_left = 0.0
        self.target_right = 0.0
        self.actual_left = 0.0
        self.actual_right = 0.0
        self.previous_curve_magnitude = {"left": 0.0, "right": 0.0}
        self.previous_curve_sign = {"left": 0, "right": 0}
        self.previous_curve_sweep = {"left": "rising", "right": "rising"}
        self.last_cmd_time = rospy.Time.now()
        self.last_update_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.update)
        self.publish_model_status()
        rospy.loginfo(
            "Drive-to-thrusters bridge initialized for namespace: %s drive=%s left=%s right=%s left_scale=%.3f right_scale=%.3f tau=%.3fs max_delta=%.3f/s empirical_model=%s",
            namespace,
            drive_topic,
            left_topic,
            right_topic,
            self.left_scale,
            self.right_scale,
            self.response_time_constant_sec,
            self.max_drive_delta_per_sec,
            "enabled" if self.empirical_model else "disabled",
        )

    def load_empirical_model(self):
        if not self.empirical_model_enabled:
            return None
        path = Path(self.empirical_model_file).expanduser()
        if not path.is_file():
            raise rospy.ROSInitException(
                "empirical simulator model enabled but unavailable: {}".format(path)
            )
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return validate_proxy(payload)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            raise rospy.ROSInitException(
                "failed to load empirical simulator model {}: {}".format(path, error)
            )

    def publish_model_status(self):
        status = {
            "enabled": self.empirical_model_enabled,
            "loaded": self.empirical_model is not None,
            "schema": "",
            "sha256": "",
            "proxy_kind": "",
        }
        if self.empirical_model is not None:
            status.update(
                {
                    "schema": self.empirical_model["schema"],
                    "sha256": payload_sha256(self.empirical_model),
                    "proxy_kind": self.empirical_model["proxy_kind"],
                    "source_model_sha256": self.empirical_model["source_model"].get(
                        "source_sha256", ""
                    ),
                    "source_model_artifact_sha256": self.empirical_model[
                        "source_model"
                    ].get("artifact_sha256", ""),
                    "path": str(Path(self.empirical_model_file).expanduser()),
                }
            )
        self.model_status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def callback(self, msg):
        # The side-specific empirical curves already own steady-state
        # asymmetry. Legacy scaling remains active only for the linear plant.
        left_scale = 1.0 if self.empirical_model else self.left_scale
        right_scale = 1.0 if self.empirical_model else self.right_scale
        self.target_left = self.shape_drive(msg.left, left_scale)
        self.target_right = self.shape_drive(msg.right, right_scale)
        self.last_cmd_time = rospy.Time.now()

    def shape_drive(self, cmd, scale):
        cmd = clamp(float(cmd), -1.0, 1.0)
        return clamp(cmd * float(scale), -1.0, 1.0)

    def drive_to_thrust(self, drive, side):
        if abs(drive) <= 1e-12:
            self.previous_curve_magnitude[side] = 0.0
            self.previous_curve_sign[side] = 0
            self.previous_curve_sweep[side] = "rising"
            return 0.0
        if self.empirical_model:
            direction = "forward" if drive >= 0.0 else "reverse"
            table = self.empirical_model["sides"][side]["directions"][direction]
            magnitude = abs(drive)
            sign = 1 if drive > 0.0 else -1
            sweep = select_hysteresis_sweep(
                self.previous_curve_sign[side],
                self.previous_curve_magnitude[side],
                self.previous_curve_sweep[side],
                sign,
                magnitude,
            )
            effort = curve_effort(table, magnitude, sweep)
            self.previous_curve_magnitude[side] = magnitude
            self.previous_curve_sign[side] = sign
            self.previous_curve_sweep[side] = sweep
            return (1.0 if drive >= 0.0 else -1.0) * effort * self.max_fwd_thrust
        if drive >= 0.0:
            return drive * self.max_fwd_thrust
        return drive * self.max_bck_thrust

    def update(self, _event):
        now = rospy.Time.now()
        dt = max(0.0, (now - self.last_update_time).to_sec())
        self.last_update_time = now
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
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

        left_wrench = Wrench()
        left_wrench.force.x = self.drive_to_thrust(self.actual_left, "left")
        self.p_left.publish(left_wrench)

        right_wrench = Wrench()
        right_wrench.force.x = self.drive_to_thrust(self.actual_right, "right")
        self.p_right.publish(right_wrench)


if __name__ == "__main__":
    try:
        DriveToThrusters()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
