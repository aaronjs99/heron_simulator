#!/usr/bin/env python3
"""Publish IG Handle-style timing topics from simulated sensor timestamps."""

from __future__ import annotations

import functools
from typing import Iterable, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, TimeReference

from models.parameters import strict_bool


def topic_list(value: object) -> list[str]:
    """Normalize ROS params that may be YAML lists or comma-separated strings."""
    if value is None:
        return []
    if isinstance(value, str):
        raw_items: Iterable[object] = value.replace(";", ",").split(",")
    elif isinstance(value, Sequence):
        raw_items = value
    else:
        raw_items = [value]
    return [str(item).strip() for item in raw_items if str(item).strip()]


def stamp_key(stamp) -> Tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


class SimIgTimingBridge(Node):
    """Bridge simulated message stamps onto the hardware timing topics."""

    def __init__(self) -> None:
        super().__init__("sim_ig_timing")

        self.pps_topic = self.declare_parameter("pps_time_topic", "/sensors/pps/time").value
        self.camera_time_topic = self.declare_parameter(
            "camera_time_topic", "/sensors/camera/time"
        ).value
        self.imu_time_topic = self.declare_parameter(
            "imu_time_topic", "/sensors/imu/time"
        ).value
        self.imu_topic = self.declare_parameter("imu_topic", "/sensors/imu/data").value
        self.camera_image_topics = topic_list(
            self.declare_parameter(
                "camera_image_topics",
                (
                    "/sensors/camera/f1/image_raw,"
                    "/sensors/camera/f2/image_raw,"
                    "/sensors/camera/f3/image_raw,"
                    "/sensors/camera/f4/image_raw"
                ),
            ).value
        )

        self.pps_rate_hz = float(self.declare_parameter("pps_rate_hz", 1.0).value)
        if self.pps_rate_hz <= 0.0:
            raise ValueError("pps_rate_hz must be positive")

        self.pps_frame_id = self.declare_parameter("pps_frame_id", "sim_pps").value
        self.default_camera_frame_id = self.declare_parameter(
            "default_camera_frame_id", "sim_camera_trigger"
        ).value
        self.default_imu_frame_id = self.declare_parameter(
            "default_imu_frame_id", "imu_link"
        ).value
        self.dedupe_camera_stamps = strict_bool(
            self.declare_parameter("dedupe_camera_stamps", True).value,
            name="dedupe_camera_stamps",
        )

        self.pps_pub = self.create_publisher(TimeReference, self.pps_topic, 10)
        self.camera_pub = self.create_publisher(TimeReference, self.camera_time_topic, 20)
        self.imu_pub = self.create_publisher(TimeReference, self.imu_time_topic, 50)

        self.last_camera_stamp: Optional[Tuple[int, int]] = None
        self.camera_subscribers = [
            self.create_subscription(
                Image, topic, functools.partial(self._camera_cb, topic=topic), 1
            )
            for topic in self.camera_image_topics
        ]
        self.imu_subscriber = self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, 10
        )
        self.pps_timer = self.create_timer(1.0 / self.pps_rate_hz, self._pps_cb)

        self.get_logger().info(
            "sim_ig_timing pps={} camera={} imu={} camera_sources={} imu_source={}".format(
                self.pps_topic,
                self.camera_time_topic,
                self.imu_time_topic,
                ",".join(self.camera_image_topics) or "<none>",
                self.imu_topic,
            )
        )

    def _valid_stamp_or_now(self, stamp):
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            return self.get_clock().now().to_msg()
        return stamp

    def _time_reference(self, stamp, frame_id: str, source: str) -> TimeReference:
        msg = TimeReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.time_ref = self._valid_stamp_or_now(stamp)
        msg.source = source
        return msg

    def _safe_publish(self, publisher, msg: TimeReference) -> None:
        if not rclpy.ok():
            return
        publisher.publish(msg)

    def _pps_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._safe_publish(
            self.pps_pub, self._time_reference(stamp, self.pps_frame_id, "sim_clock")
        )

    def _camera_cb(self, msg: Image, topic: str) -> None:
        stamp = self._valid_stamp_or_now(msg.header.stamp)
        key = stamp_key(stamp)
        if self.dedupe_camera_stamps and key == self.last_camera_stamp:
            return
        self.last_camera_stamp = key
        frame_id = msg.header.frame_id or self.default_camera_frame_id
        self._safe_publish(
            self.camera_pub,
            self._time_reference(stamp, frame_id, "sim_camera:" + topic),
        )

    def _imu_cb(self, msg: Imu) -> None:
        stamp = self._valid_stamp_or_now(msg.header.stamp)
        frame_id = msg.header.frame_id or self.default_imu_frame_id
        self._safe_publish(
            self.imu_pub, self._time_reference(stamp, frame_id, "sim_imu")
        )


def main() -> None:
    rclpy.init()
    node = SimIgTimingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()