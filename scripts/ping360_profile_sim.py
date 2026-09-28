#!/usr/bin/env python3
"""Publish canonical Ping360 profiles from a full-circle Gazebo ray cloud."""

from __future__ import annotations

import hashlib
import math
import struct
import time
from copy import deepcopy

import rclpy
from ig_handle.msg import SonarProfile
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from models.ping360_profile_model import (
    MechanicalSweep,
    profile_from_points,
)


class Ping360ProfileSimulator(Node):
    def __init__(self):
        super().__init__("ping360_profile_sim")
        self.input_topic = str(
            self._parameter("input_topic", "/sim/sensors/sonar/imaging/rays")
        )
        self.profile_topic = str(
            self._parameter("profile_topic", "/sensors/sonar/imaging/profile")
        )
        self.frame_id = (
            str(self._parameter("frame_id", "ping360_link")).strip().lstrip("/")
        )
        self.extrinsic_revision = str(
            self._parameter("extrinsic_revision", "") or ""
        ).strip()
        if not self.extrinsic_revision:
            raise ValueError("~extrinsic_revision is required")
        if not self.frame_id:
            raise ValueError("~frame_id is required")
        self.provider = str(
            self._parameter("provider", "blue_robotics_ping360")
        ).strip()
        self.model = str(self._parameter("model", "Ping360")).strip()
        if not self.provider or not self.model:
            raise ValueError("~provider and ~model are required")
        self.sound_speed_mps = float(self._parameter("sound_speed_mps", 1480.0))
        self.number_of_samples = int(self._parameter("number_of_samples", 1200))
        self.min_range_m = float(self._parameter("min_range_m", 0.5))
        self.max_range_m = float(self._parameter("max_range_m", 100.0))
        self.gain_setting = int(self._parameter("gain_setting", 1))
        self.transmit_duration_us = int(self._parameter("transmit_duration_us", 11))
        self.transmit_frequency_khz = int(
            self._parameter("transmit_frequency_khz", 750)
        )
        self.drop_every_n = int(self._parameter("drop_every_n", 0))
        self.invalid_every_n = int(self._parameter("invalid_every_n", 0))
        self.sweep = MechanicalSweep(
            int(self._parameter("start_angle_grad", 0)),
            int(self._parameter("stop_angle_grad", 399)),
            int(self._parameter("num_steps", 1)),
        )
        self.sample_interval_m = self.max_range_m / self.number_of_samples
        self.sample_period_ticks = int(
            round(self.sample_interval_m * 2.0 / (self.sound_speed_mps * 25e-9))
        )
        if not 80 <= self.sample_period_ticks <= 40000:
            raise ValueError("simulated sample period is outside Ping360 limits")
        self.sequence = 0
        self.last_frame_warning_wall_sec = -float("inf")
        self.publisher = self.create_publisher(SonarProfile, self.profile_topic, 20)
        self.subscriber = self.create_subscription(
            LaserScan, self.input_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "ping360_profile_sim input=%s profile=%s frame=%s revision=%s"
            % (
                self.input_topic,
                self.profile_topic,
                self.frame_id,
                self.extrinsic_revision,
            )
        )

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _scan_callback(self, scan: LaserScan) -> None:
        source_frame = str(scan.header.frame_id or "").lstrip("/")
        expected_frame = self.frame_id.lstrip("/")
        if source_frame != expected_frame:
            now = time.monotonic()
            if now - self.last_frame_warning_wall_sec >= 5.0:
                self.get_logger().warning(
                    "ping360_profile_sim dropped profile: source frame '%s' != '%s'"
                    % (source_frame or "(empty)", expected_frame)
                )
                self.last_frame_warning_wall_sec = now
            return
        self.sequence += 1
        angle_grad = self.sweep.advance()
        if self.drop_every_n and self.sequence % self.drop_every_n == 0:
            return
        points = []
        for index, measured_range in enumerate(scan.ranges):
            range_m = float(measured_range)
            if (
                not math.isfinite(range_m)
                or not self.min_range_m <= range_m <= self.max_range_m
            ):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            intensity = (
                float(scan.intensities[index])
                if index < len(scan.intensities)
                and math.isfinite(scan.intensities[index])
                else 0.0
            )
            points.append(
                (range_m * math.cos(angle), range_m * math.sin(angle), 0.0, intensity)
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
                float(scan.header.stamp.sec)
                + float(scan.header.stamp.nanosec) * 1.0e-9,
                angle_grad,
                self.sample_period_ticks,
                self.transmit_frequency_khz,
                self.number_of_samples,
            )
            + intensities
        )
        msg = SonarProfile()
        msg.header = deepcopy(scan.header)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
