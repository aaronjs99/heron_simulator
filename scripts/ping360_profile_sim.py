#!/usr/bin/env python3
"""Publish canonical Ping360 profiles from a full-circle Gazebo ray cloud."""

from __future__ import annotations

import hashlib
import struct

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from ig_handle.msg import SonarProfile
from sensor_msgs.msg import PointCloud2

from models.ping360_profile_model import (
    MechanicalSweep,
    profile_from_points,
)


class Ping360ProfileSimulator(Node):
    def __init__(self):
        super().__init__("ping360_profile_sim")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/sim/sensors/sonar/ping360_points").value
        )
        self.profile_topic = str(
            self.declare_parameter("profile_topic", "/sensors/sonar/imaging/profile").value
        )
        self.frame_id = (
            str(self.declare_parameter("frame_id", "ping360_link").value).strip().lstrip("/")
        )
        self.extrinsic_revision = str(
            self.declare_parameter("extrinsic_revision", "").value or ""
        ).strip()
        if not self.extrinsic_revision:
            raise ValueError("extrinsic_revision is required")
        if not self.frame_id:
            raise ValueError("frame_id is required")
        self.provider = str(
            self.declare_parameter("provider", "blue_robotics_ping360").value
        ).strip()
        self.model = str(self.declare_parameter("model", "Ping360").value).strip()
        if not self.provider or not self.model:
            raise ValueError("provider and model are required")
        self.sound_speed_mps = float(self.declare_parameter("sound_speed_mps", 1480.0).value)
        self.number_of_samples = int(self.declare_parameter("number_of_samples", 1200).value)
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.5).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 100.0).value)
        self.gain_setting = int(self.declare_parameter("gain_setting", 1).value)
        self.transmit_duration_us = int(self.declare_parameter("transmit_duration_us", 11).value)
        self.transmit_frequency_khz = int(
            self.declare_parameter("transmit_frequency_khz", 750).value
        )
        self.drop_every_n = int(self.declare_parameter("drop_every_n", 0).value)
        self.invalid_every_n = int(self.declare_parameter("invalid_every_n", 0).value)
        self.sweep = MechanicalSweep(
            int(self.declare_parameter("start_angle_grad", 0).value),
            int(self.declare_parameter("stop_angle_grad", 399).value),
            int(self.declare_parameter("num_steps", 1).value),
        )
        self.sample_interval_m = self.max_range_m / self.number_of_samples
        self.sample_period_ticks = int(
            round(self.sample_interval_m * 2.0 / (self.sound_speed_mps * 25e-9))
        )
        if not 80 <= self.sample_period_ticks <= 40000:
            raise ValueError("simulated sample period is outside Ping360 limits")
        self.sequence = 0
        self.publisher = self.create_publisher(SonarProfile, self.profile_topic, 20)
        self.subscriber = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, 2
        )
        self.get_logger().info(
            "ping360_profile_sim input={} profile={} frame={} revision={}".format(
                self.input_topic, self.profile_topic, self.frame_id, self.extrinsic_revision
            )
        )

    def _cloud_callback(self, cloud):
        source_frame = str(cloud.header.frame_id or "").lstrip("/")
        expected_frame = self.frame_id.lstrip("/")
        if source_frame != expected_frame:
            self.get_logger().warn(
                "ping360_profile_sim dropped profile: source frame '{}' != '{}'".format(
                    source_frame or "(empty)", expected_frame
                ),
                throttle_duration_sec=5.0,
            )
            return
        self.sequence += 1
        angle_grad = self.sweep.advance()
        if self.drop_every_n and self.sequence % self.drop_every_n == 0:
            return
        fields = {field.name for field in cloud.fields}
        selected = (
            ("x", "y", "z", "intensity") if "intensity" in fields else ("x", "y", "z")
        )
        points = []
        for point in pc2.read_points(cloud, field_names=selected, skip_nans=True):
            intensity = float(point[3]) if len(point) == 4 else 0.0
            points.append(
                (float(point[0]), float(point[1]), float(point[2]), intensity)
            )
        intensities = profile_from_points(
            points,
            angle_grad=angle_grad,
            number_of_samples=self.number_of_samples,
            sample_interval_m=self.sample_interval_m,
        )
        invalid = bool(
            self.invalid_every_n and self.sequence % self.invalid_every_n == 0
        )
        # Raw builtin_interfaces/Time fields (sec, nanosec) are unaffected by
        # the ROS2 Header changes - only 'seq' was removed 
        stamp_sec = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        identity = (
            self.provider.encode("utf-8")
            + b"\0"
            + self.model.encode("utf-8")
            + b"\0"
            + self.frame_id.encode("utf-8")
            + b"\0"
            + self.extrinsic_revision.encode("utf-8")
            + b"\0"
            + struct.pack(
                "<IdHHHH",
                self.sequence,
                stamp_sec,
                angle_grad,
                self.sample_period_ticks,
                self.transmit_frequency_khz,
                self.number_of_samples,
            )
            + intensities
        )
        msg = SonarProfile()
        msg.header = cloud.header
        msg.header.frame_id = self.frame_id
        msg.profile_id = hashlib.sha256(identity).hexdigest()
        msg.provider = self.provider
        msg.model = self.model
        msg.extrinsic_revision = self.extrinsic_revision
        msg.synthetic = True
        msg.raw_packet_id = ""
        msg.sequence = self.sequence
        msg.valid = not invalid
        msg.validity_reason = (
            "deterministic_malformed_fixture" if invalid else "gazebo_ray_profile"
        )
        msg.angle_rad = angle_grad * 0.9 * 3.141592653589793 / 180.0
        msg.angle_grad = angle_grad
        msg.auto_scan = True
        msg.start_angle_grad = self.sweep.start
        msg.stop_angle_grad = self.sweep.stop
        msg.num_steps = self.sweep.step
        msg.gain_setting = self.gain_setting
        msg.transmit_duration_us = self.transmit_duration_us
        msg.sample_period_ticks_25ns = self.sample_period_ticks
        msg.transmit_frequency_khz = self.transmit_frequency_khz
        msg.sound_speed_mps = self.sound_speed_mps
        msg.sample_interval_m = self.sample_interval_m
        # Ping Protocol bins start at zero range. ``min_range_m`` above is only
        # a simulator return gate; publishing it as a bin origin would add a
        # systematic offset to every reconstructed return.
        msg.min_range_m = 0.0
        msg.max_range_m = self.max_range_m
        msg.intensities = [] if invalid else list(intensities)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Ping360ProfileSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()