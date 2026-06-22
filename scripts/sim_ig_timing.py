#!/usr/bin/env python3
"""Publish IG Handle-style timing topics from simulated sensor timestamps."""

from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

import rospy
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image, Imu, TimeReference


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


def valid_stamp_or_now(stamp: rospy.Time) -> rospy.Time:
    if stamp is None or stamp.to_nsec() == 0:
        return rospy.Time.now()
    return stamp


def stamp_key(stamp: rospy.Time) -> Tuple[int, int]:
    return int(stamp.secs), int(stamp.nsecs)


class SimIgTimingBridge:
    """Bridge simulated message stamps onto the hardware timing topics."""

    def __init__(self) -> None:
        self.pps_topic = rospy.get_param("~pps_time_topic", "/sensors/pps/time")
        self.camera_time_topic = rospy.get_param(
            "~camera_time_topic", "/sensors/camera/time"
        )
        self.imu_time_topic = rospy.get_param("~imu_time_topic", "/sensors/imu/time")
        self.imu_topic = rospy.get_param("~imu_topic", "/sensors/imu/data")
        self.camera_image_topics = topic_list(
            rospy.get_param(
                "~camera_image_topics",
                (
                    "/sensors/camera/f1/image_raw,"
                    "/sensors/camera/f2/image_raw,"
                    "/sensors/camera/f3/image_raw,"
                    "/sensors/camera/f4/image_raw"
                ),
            )
        )

        self.pps_rate_hz = float(rospy.get_param("~pps_rate_hz", 1.0))
        if self.pps_rate_hz <= 0.0:
            raise ValueError("~pps_rate_hz must be positive")

        self.pps_frame_id = rospy.get_param("~pps_frame_id", "sim_pps")
        self.default_camera_frame_id = rospy.get_param(
            "~default_camera_frame_id", "sim_camera_trigger"
        )
        self.default_imu_frame_id = rospy.get_param("~default_imu_frame_id", "imu_link")
        self.dedupe_camera_stamps = bool(rospy.get_param("~dedupe_camera_stamps", True))

        self.pps_pub = rospy.Publisher(self.pps_topic, TimeReference, queue_size=10)
        self.camera_pub = rospy.Publisher(
            self.camera_time_topic, TimeReference, queue_size=20
        )
        self.imu_pub = rospy.Publisher(
            self.imu_time_topic, TimeReference, queue_size=50
        )

        self.last_camera_stamp: Optional[Tuple[int, int]] = None
        self.camera_subscribers = [
            rospy.Subscriber(
                topic, Image, self._camera_cb, callback_args=topic, queue_size=1
            )
            for topic in self.camera_image_topics
        ]
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self._imu_cb, queue_size=10
        )
        self.pps_timer = rospy.Timer(
            rospy.Duration(1.0 / self.pps_rate_hz), self._pps_cb
        )

        rospy.loginfo(
            "sim_ig_timing pps=%s camera=%s imu=%s camera_sources=%s imu_source=%s",
            self.pps_topic,
            self.camera_time_topic,
            self.imu_time_topic,
            ",".join(self.camera_image_topics) or "<none>",
            self.imu_topic,
        )

    def _time_reference(
        self, stamp: rospy.Time, frame_id: str, source: str
    ) -> TimeReference:
        msg = TimeReference()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.time_ref = valid_stamp_or_now(stamp)
        msg.source = source
        return msg

    def _safe_publish(self, publisher: rospy.Publisher, msg: TimeReference) -> None:
        if rospy.is_shutdown():
            return
        try:
            publisher.publish(msg)
        except ROSException as exc:
            if rospy.is_shutdown() or "closed topic" in str(exc).lower():
                return
            raise

    def _pps_cb(self, _event: rospy.timer.TimerEvent) -> None:
        stamp = rospy.Time.now()
        self._safe_publish(
            self.pps_pub, self._time_reference(stamp, self.pps_frame_id, "sim_clock")
        )

    def _camera_cb(self, msg: Image, topic: str) -> None:
        stamp = valid_stamp_or_now(msg.header.stamp)
        key = stamp_key(stamp)
        if self.dedupe_camera_stamps and key == self.last_camera_stamp:
            return
        self.last_camera_stamp = key
        frame_id = msg.header.frame_id or self.default_camera_frame_id
        self._safe_publish(
            self.camera_pub, self._time_reference(stamp, frame_id, "sim_camera:" + topic)
        )

    def _imu_cb(self, msg: Imu) -> None:
        stamp = valid_stamp_or_now(msg.header.stamp)
        frame_id = msg.header.frame_id or self.default_imu_frame_id
        self._safe_publish(
            self.imu_pub, self._time_reference(stamp, frame_id, "sim_imu")
        )


def main() -> None:
    rospy.init_node("sim_ig_timing")
    SimIgTimingBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
