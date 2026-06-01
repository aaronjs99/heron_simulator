#!/usr/bin/env python3
"""Publish raw multibeam echosounder profile packets from Gazebo rays."""

from __future__ import annotations

import math
import struct
from typing import List, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8MultiArray


PointRecord = Tuple[float, float, float, int]
XYZ_RECORD = struct.Struct("<fffH")


def encode_profile_packet(
    points: Sequence[PointRecord],
    *,
    header_bytes: int = 256,
    packet_kind: bytes = b"83P",
) -> bytes:
    """Encode XYZ profile records in the packet shape used by ig_handle."""

    if header_bytes < len(packet_kind):
        raise ValueError("header_bytes must fit packet_kind")

    header = bytearray(header_bytes)
    header[: len(packet_kind)] = packet_kind
    if header_bytes >= 24:
        header[8:24] = b"SIM_MULTIBEAM_83P"[:16]

    if not points:
        return bytes(header) + (b"\0" * XYZ_RECORD.size)

    payload = bytearray()
    for x, y, z, intensity in points:
        payload.extend(
            XYZ_RECORD.pack(
                float(x),
                float(y),
                float(z),
                max(0, min(65535, int(intensity))),
            )
        )
    return bytes(header) + bytes(payload)


def point_records_from_cloud(
    cloud: PointCloud2,
    *,
    max_points: int,
    min_range_m: float,
    max_range_m: float,
) -> List[PointRecord]:
    """Extract finite sonar-frame returns from a Gazebo ray PointCloud2."""

    field_names = {field.name for field in cloud.fields}
    read_fields = (
        ("x", "y", "z", "intensity") if "intensity" in field_names else ("x", "y", "z")
    )
    min_range_sq = float(min_range_m) * float(min_range_m)
    max_range_sq = float(max_range_m) * float(max_range_m)
    records: List[PointRecord] = []

    for point in pc2.read_points(cloud, field_names=read_fields, skip_nans=True):
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        range_sq = x * x + y * y + z * z
        if range_sq < min_range_sq or range_sq > max_range_sq:
            continue
        if len(read_fields) == 4 and math.isfinite(float(point[3])):
            intensity = int(point[3])
        else:
            intensity = 0
        records.append((x, y, z, intensity))
        if len(records) >= max_points:
            break

    return records


class MultibeamRawNode:
    """Bridge Gazebo's internal multibeam ray cloud into raw profile packets."""

    def __init__(self) -> None:
        self.input_topic = rospy.get_param(
            "~input_topic", "/sim/sensors/sonar/multibeam_points"
        )
        self.raw_topic = rospy.get_param("~raw_topic", "/sensors/sonar/raw")
        self.header_bytes = int(rospy.get_param("~header_bytes", 256))
        self.packet_kind = str(rospy.get_param("~packet_kind", "83P")).encode("ascii")
        self.max_points = int(rospy.get_param("~max_points", 480))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.5))
        self.max_range_m = float(rospy.get_param("~max_range_m", 100.0))

        self.publisher = rospy.Publisher(self.raw_topic, UInt8MultiArray, queue_size=20)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self._cloud_cb, queue_size=5
        )
        rospy.loginfo(
            "multibeam_raw input=%s raw=%s beams=%d range=[%.2f, %.2f]",
            self.input_topic,
            self.raw_topic,
            self.max_points,
            self.min_range_m,
            self.max_range_m,
        )

    def _cloud_cb(self, cloud: PointCloud2) -> None:
        points = point_records_from_cloud(
            cloud,
            max_points=self.max_points,
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
        )
        packet = encode_profile_packet(
            points,
            header_bytes=self.header_bytes,
            packet_kind=self.packet_kind,
        )
        msg = UInt8MultiArray()
        msg.data = list(packet)
        self.publisher.publish(msg)
        rospy.logdebug(
            "multibeam_raw packet_kind=%s points=%d bytes=%d",
            self.packet_kind.decode("ascii", errors="replace"),
            len(points),
            len(packet),
        )


def main() -> None:
    rospy.init_node("multibeam_raw")
    MultibeamRawNode()
    rospy.spin()


if __name__ == "__main__":
    main()
