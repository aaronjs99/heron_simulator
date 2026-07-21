#!/usr/bin/env python3
"""Publish canonical Ping360 profiles from a full-circle Gazebo ray cloud."""

from __future__ import annotations

import hashlib
import struct
import sys
from pathlib import Path

import rospy
import sensor_msgs.point_cloud2 as pc2
from ig_handle.msg import SonarProfile
from sensor_msgs.msg import PointCloud2

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ping360_profile_model import MechanicalSweep, profile_from_points  # noqa: E402


class Ping360ProfileSimulator:
    def __init__(self):
        self.input_topic = str(
            rospy.get_param("~input_topic", "/sim/sensors/sonar/ping360_points")
        )
        self.profile_topic = str(
            rospy.get_param("~profile_topic", "/sensors/sonar/imaging/profile")
        )
        self.frame_id = str(rospy.get_param("~frame_id", "sonar_link"))
        self.sound_speed_mps = float(rospy.get_param("~sound_speed_mps", 1480.0))
        self.number_of_samples = int(rospy.get_param("~number_of_samples", 1200))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.5))
        self.max_range_m = float(rospy.get_param("~max_range_m", 100.0))
        self.gain_setting = int(rospy.get_param("~gain_setting", 1))
        self.transmit_duration_us = int(rospy.get_param("~transmit_duration_us", 11))
        self.transmit_frequency_khz = int(
            rospy.get_param("~transmit_frequency_khz", 750)
        )
        self.drop_every_n = int(rospy.get_param("~drop_every_n", 0))
        self.invalid_every_n = int(rospy.get_param("~invalid_every_n", 0))
        self.sweep = MechanicalSweep(
            int(rospy.get_param("~start_angle_grad", 0)),
            int(rospy.get_param("~stop_angle_grad", 399)),
            int(rospy.get_param("~num_steps", 1)),
        )
        self.sample_interval_m = self.max_range_m / self.number_of_samples
        self.sample_period_ticks = int(
            round(self.sample_interval_m * 2.0 / (self.sound_speed_mps * 25e-9))
        )
        if not 80 <= self.sample_period_ticks <= 40000:
            raise ValueError("simulated sample period is outside Ping360 limits")
        self.sequence = 0
        self.publisher = rospy.Publisher(
            self.profile_topic, SonarProfile, queue_size=20
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self._cloud_callback, queue_size=2
        )

    def _cloud_callback(self, cloud):
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
        identity = (
            struct.pack(
                "<IdHHHH",
                self.sequence,
                cloud.header.stamp.to_sec(),
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
        msg.provider = "simulated_ping360"
        msg.model = "Ping360_canonical_sim"
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
        msg.min_range_m = self.min_range_m
        msg.max_range_m = self.max_range_m
        msg.intensities = [] if invalid else list(intensities)
        self.publisher.publish(msg)


def main():
    rospy.init_node("ping360_profile_sim")
    Ping360ProfileSimulator()
    rospy.spin()


if __name__ == "__main__":
    main()
