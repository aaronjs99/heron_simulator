#!/usr/bin/env python3
"""Publish IG Handle-style timing topics from simulated sensor timestamps."""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, TimeReference

from models.parameters import strict_bool


def topic_list(value: object) -> list[str]:
    """Normalize ROS parameters that may be lists or comma-separated strings."""
    if value is None:
        return []
    if isinstance(value, str):
        raw_items: Iterable[object] = value.replace(";", ",").split(",")
    elif isinstance(value, Sequence):
        raw_items = value
    else:
        raw_items = [value]
    return [str(item).strip() for item in raw_items if str(item).strip()]


def valid_stamp_or_now(stamp: Time, now: Time) -> Time:
    if stamp is None or (int(stamp.sec) == 0 and int(stamp.nanosec) == 0):
        return now
    return stamp


def stamp_key(stamp: Time) -> Tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


class SimIgTimingBridge(Node):
    """Bridge simulated message stamps onto the hardware timing topics."""

    def __init__(self) -> None:
        super().__init__("sim_ig_timing")

        self.pps_topic = str(self._parameter("pps_time_topic", "/sensors/pps/time"))
        self.camera_time_topic = str(
            self._parameter("camera_time_topic", "/sensors/camera/time")
        )
        self.imu_time_topic = str(
            self._parameter("imu_time_topic", "/sensors/imu/time")
        )
        self.imu_topic = str(self._parameter("imu_topic", "/sensors/imu/data"))
        self.camera_image_topics = topic_list(
            self._parameter(
                "camera_image_topics",
                "/sensors/camera/f1/image_raw,"
                "/sensors/camera/f2/image_raw,"
                "/sensors/camera/f3/image_raw,"
                "/sensors/camera/f4/image_raw",
            )
        )

        self.pps_rate_hz = float(self._parameter("pps_rate_hz", 1.0))
        if self.pps_rate_hz <= 0.0:
            raise ValueError("pps_rate_hz must be positive")

        self.pps_frame_id = str(self._parameter("pps_frame_id", "sim_pps"))
        self.default_camera_frame_id = str(
            self._parameter("default_camera_frame_id", "sim_camera_trigger")
        )
        self.default_imu_frame_id = str(
            self._parameter("default_imu_frame_id", "imu_link")
        )
        self.dedupe_camera_stamps = strict_bool(
            self._parameter("dedupe_camera_stamps", True),
            name="dedupe_camera_stamps",
        )

        self.pps_pub = self.create_publisher(TimeReference, self.pps_topic, 10)
        self.camera_pub = self.create_publisher(
            TimeReference, self.camera_time_topic, 20
        )
        self.imu_pub = self.create_publisher(TimeReference, self.imu_time_topic, 50)

        self.last_camera_stamp: Tuple[int, int] | None = None
        self.camera_subscribers = [
            self.create_subscription(
                Image,
                topic,
                lambda msg, source=topic: self._camera_cb(msg, source),
                qos_profile_sensor_data,
            )
            for topic in self.camera_image_topics
        ]
        self.imu_subscriber = self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data
        )
        self.pps_timer = self.create_timer(1.0 / self.pps_rate_hz, self._pps_cb)

        self.get_logger().info(
            "sim_ig_timing pps=%s camera=%s imu=%s camera_sources=%s imu_source=%s"
            % (
                self.pps_topic,
                self.camera_time_topic,
                self.imu_time_topic,
                ",".join(self.camera_image_topics) or "<none>",
                self.imu_topic,
            )
        )

    def _parameter(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _time_reference(self, stamp: Time, frame_id: str, source: str) -> TimeReference:
        msg = TimeReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.time_ref = valid_stamp_or_now(stamp, self.get_clock().now().to_msg())
        msg.source = source
        return msg

    def _pps_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.pps_pub.publish(
            self._time_reference(stamp, self.pps_frame_id, "sim_clock")
        )

    def _camera_cb(self, msg: Image, topic: str) -> None:
        stamp = valid_stamp_or_now(msg.header.stamp, self.get_clock().now().to_msg())
        key = stamp_key(stamp)
        if self.dedupe_camera_stamps and key == self.last_camera_stamp:
            return
        self.last_camera_stamp = key
        frame_id = msg.header.frame_id or self.default_camera_frame_id
        self.camera_pub.publish(
            self._time_reference(stamp, frame_id, "sim_camera:" + topic)
        )

    def _imu_cb(self, msg: Imu) -> None:
        stamp = valid_stamp_or_now(msg.header.stamp, self.get_clock().now().to_msg())
        frame_id = msg.header.frame_id or self.default_imu_frame_id
        self.imu_pub.publish(self._time_reference(stamp, frame_id, "sim_imu"))


def main() -> None:
    rclpy.init()
    node = SimIgTimingBridge()
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