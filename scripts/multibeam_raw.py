#!/usr/bin/env python3
"""Encode Gazebo DT100 rays as strict Imagenex 83P profile datagrams."""

from __future__ import annotations

import math
import struct
from datetime import datetime, timezone
from typing import List, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from ig_handle.msg import SonarRawPacket as SonarRawPacketMessage
from rospy.exceptions import ROSException
from sensor_msgs.msg import PointCloud2

from models.parameters import strict_bool

PointRecord = Tuple[float, float, float, int]
HEADER_BYTES = 256
PACKET_KIND = b"83P"
PACKET_VERSION = 10
HIGH_RESOLUTION_MAX_SAMPLE = 4999
LATENCY_UNIT_SEC = 1.0e-4


def _u16(value: int, field_name: str) -> int:
    value = int(value)
    if not 0 <= value <= 0xFFFF:
        raise ValueError("{} must fit an unsigned 16-bit field".format(field_name))
    return value


def _timestamp_fields(header: bytearray, interrogation_time_sec: float) -> None:
    stamp = datetime.fromtimestamp(
        max(0.0, float(interrogation_time_sec)), tz=timezone.utc
    )
    header[8:20] = stamp.strftime("%d-%b-%Y").upper().encode("ascii") + b"\0"
    header[20:29] = stamp.strftime("%H:%M:%S").encode("ascii") + b"\0"
    header[29:33] = (".%02d" % (stamp.microsecond // 10000)).encode("ascii") + b"\0"
    header[112:117] = (".%03d" % (stamp.microsecond // 1000)).encode("ascii") + b"\0"


def beam_profile_from_points(
    points: Sequence[PointRecord],
    *,
    beam_count: int,
    start_angle_deg: float,
    sector_size_deg: float,
    angle_increment_deg: float,
    range_resolution_mm: int,
    sound_speed_m_s: float,
    min_range_m: float,
    max_range_m: float,
) -> Tuple[List[int], List[int]]:
    """Assign the nearest valid Gazebo return to each DT100 profile beam."""

    beam_count = int(beam_count)
    if beam_count <= 0:
        raise ValueError("beam_count must be positive")
    if sector_size_deg <= 0.0 or sector_size_deg > 360.0:
        raise ValueError("sector_size_deg must be in (0, 360]")
    if angle_increment_deg <= 0.0:
        raise ValueError("angle_increment_deg must be positive")
    if (beam_count - 1) * angle_increment_deg > sector_size_deg + 0.01:
        raise ValueError("beam centers exceed sector_size_deg")
    if range_resolution_mm <= 0:
        raise ValueError("range_resolution_mm must be positive")
    if sound_speed_m_s <= 0.0:
        raise ValueError("sound_speed_m_s must be positive")
    if min_range_m < 0.0 or max_range_m < min_range_m:
        raise ValueError("invalid range gate")

    ranges = [0] * beam_count
    intensities = [0] * beam_count
    nearest_ranges = [math.inf] * beam_count
    range_per_sample_m = (
        float(range_resolution_mm) / 1000.0 * float(sound_speed_m_s) / 1500.0
    )
    source_increment_deg = float(angle_increment_deg)

    for x, y, z, intensity in points:
        x, y, z = float(x), float(y), float(z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        range_m = math.sqrt(x * x + y * y + z * z)
        if range_m < min_range_m or range_m > max_range_m:
            continue

        angle_deg = math.degrees(math.atan2(y, x))
        beam_index = int(round((angle_deg - start_angle_deg) / source_increment_deg))
        if not 0 <= beam_index < beam_count:
            continue
        expected_angle_deg = start_angle_deg + beam_index * source_increment_deg
        if abs(angle_deg - expected_angle_deg) > source_increment_deg * 0.51:
            continue
        if range_m >= nearest_ranges[beam_index]:
            continue

        sample = int(round(range_m / range_per_sample_m))
        if not 1 <= sample <= HIGH_RESOLUTION_MAX_SAMPLE:
            continue
        ranges[beam_index] = sample
        intensities[beam_index] = max(0, min(0xFFFF, int(round(intensity))))
        nearest_ranges[beam_index] = range_m

    return ranges, intensities


def encode_profile_packet(
    ranges: Sequence[int],
    intensities: Sequence[int],
    *,
    interrogation_time_sec: float,
    samples_per_beam: int,
    sector_size_deg: float,
    start_angle_deg: float,
    angle_increment_deg: float,
    acoustic_range_m: float,
    acoustic_frequency_khz: int,
    sound_speed_m_s: float,
    range_resolution_mm: int,
    ping_number: int,
    ping_latency_units: int = 0,
    data_latency_units: int = 0,
    center_ping_offset_units: int = 0,
    include_intensity: bool = True,
) -> bytes:
    """Encode one exact-length Imagenex 83P v1.10 datagram.

    All multibyte integer fields and profile samples use network byte order as
    specified by Imagenex. ``*_latency_units`` are 100 microsecond ticks.
    """

    beam_count = len(ranges)
    if beam_count <= 0:
        raise ValueError("at least one profile beam is required")
    if len(intensities) != beam_count:
        raise ValueError("intensities must align one-to-one with ranges")
    if not -180.0 <= start_angle_deg <= 180.0:
        raise ValueError("start_angle_deg must be in [-180, 180]")
    angle_increment_hundredths = int(round(float(angle_increment_deg) * 100.0))
    if not 1 <= angle_increment_hundredths <= 0xFF:
        raise ValueError("angle_increment_deg is not representable in byte 78")
    encoded_increment_deg = angle_increment_hundredths / 100.0
    if (beam_count - 1) * encoded_increment_deg > float(sector_size_deg) + 0.01:
        raise ValueError("beam angles exceed sector_size_deg")
    sound_speed_tenths = int(round(float(sound_speed_m_s) * 10.0))
    if not 1 <= sound_speed_tenths <= 0x7FFF:
        raise ValueError("sound_speed_m_s is not representable in bytes 83-84")

    intensity_bytes = 2 * beam_count if include_intensity else 0
    total_bytes = HEADER_BYTES + 2 * beam_count + intensity_bytes
    _u16(total_bytes, "total_bytes")
    header = bytearray(HEADER_BYTES)
    header[0:3] = PACKET_KIND
    header[3] = PACKET_VERSION
    struct.pack_into(">H", header, 4, total_bytes)
    _timestamp_fields(header, interrogation_time_sec)
    struct.pack_into(">H", header, 70, _u16(beam_count, "beam_count"))
    struct.pack_into(">H", header, 72, _u16(samples_per_beam, "samples_per_beam"))
    struct.pack_into(">H", header, 74, _u16(round(sector_size_deg), "sector_size_deg"))
    struct.pack_into(
        ">H",
        header,
        76,
        _u16(round((float(start_angle_deg) + 180.0) * 100.0), "start_angle"),
    )
    header[78] = angle_increment_hundredths
    struct.pack_into(
        ">H", header, 79, _u16(round(acoustic_range_m), "acoustic_range_m")
    )
    struct.pack_into(
        ">H",
        header,
        81,
        _u16(acoustic_frequency_khz, "acoustic_frequency_khz"),
    )
    struct.pack_into(">H", header, 83, 0x8000 | sound_speed_tenths)
    struct.pack_into(">H", header, 85, _u16(range_resolution_mm, "range_resolution_mm"))
    struct.pack_into(">H", header, 89, 180)
    struct.pack_into(">I", header, 93, int(ping_number) & 0xFFFFFFFF)
    header[117] = 1 if include_intensity else 0
    struct.pack_into(">H", header, 118, _u16(ping_latency_units, "ping_latency_units"))
    struct.pack_into(">H", header, 120, _u16(data_latency_units, "data_latency_units"))
    header[122] = 1
    header[125] = 1
    struct.pack_into(
        ">H",
        header,
        126,
        _u16(center_ping_offset_units, "center_ping_offset_units"),
    )

    payload = bytearray()
    for sample in ranges:
        sample = int(sample)
        if not 0 <= sample <= HIGH_RESOLUTION_MAX_SAMPLE:
            raise ValueError("high-resolution profile samples must be in [0, 4999]")
        payload.extend(struct.pack(">H", sample))
    if include_intensity:
        for intensity in intensities:
            payload.extend(struct.pack(">H", _u16(intensity, "intensity")))

    packet = bytes(header) + bytes(payload)
    if len(packet) != total_bytes:
        raise AssertionError("internal 83P length mismatch")
    return packet


def point_records_from_cloud(
    cloud: PointCloud2,
    *,
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
        intensity = (
            int(point[3])
            if len(read_fields) == 4 and math.isfinite(float(point[3]))
            else 0
        )
        records.append((x, y, z, intensity))

    return records


class MultibeamRawNode:
    """Bridge Gazebo's DT100 ray profile into the hardware raw-data contract."""

    def __init__(self) -> None:
        self.input_topic = rospy.get_param(
            "~input_topic", "/sim/sensors/sonar/multibeam_points"
        )
        self.raw_topic = rospy.get_param("~raw_topic", "/sensors/sonar/raw")
        self.frame_id = (
            str(rospy.get_param("~frame_id", "dt100_link")).strip().lstrip("/")
        )
        self.extrinsic_revision = str(
            rospy.get_param("~extrinsic_revision", "") or ""
        ).strip()
        if not self.frame_id or not self.extrinsic_revision:
            raise ValueError("~frame_id and ~extrinsic_revision are required")
        self.provider = str(rospy.get_param("~provider", "imagenex_dt100")).strip()
        self.model = str(rospy.get_param("~model", "Imagenex DT100")).strip()
        if not self.provider or not self.model:
            raise ValueError("~provider and ~model are required")
        self.source_endpoint = str(
            rospy.get_param("~source_endpoint", "gazebo://dt100")
        )
        self.beam_count = int(rospy.get_param("~beam_count", 480))
        self.samples_per_beam = int(rospy.get_param("~samples_per_beam", 5000))
        self.sector_size_deg = float(rospy.get_param("~sector_size_deg", 120.0))
        self.start_angle_deg = float(rospy.get_param("~start_angle_deg", -60.0))
        self.angle_increment_deg = float(
            rospy.get_param(
                "~angle_increment_deg", self.sector_size_deg / self.beam_count
            )
        )
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.5))
        self.max_range_m = float(rospy.get_param("~max_range_m", 100.0))
        self.range_resolution_mm = int(rospy.get_param("~range_resolution_mm", 20))
        self.acoustic_frequency_khz = int(
            rospy.get_param("~acoustic_frequency_khz", 240)
        )
        self.sound_speed_m_s = float(rospy.get_param("~sound_speed_m_s", 1500.0))
        self.ping_latency_units = _u16(
            rospy.get_param("~ping_latency_units", 0), "ping_latency_units"
        )
        self.center_ping_offset_units = _u16(
            rospy.get_param("~center_ping_offset_units", 0),
            "center_ping_offset_units",
        )
        self.include_intensity = strict_bool(
            rospy.get_param("~include_intensity", True), name="~include_intensity"
        )
        self.sequence = 0

        self.publisher = rospy.Publisher(
            self.raw_topic, SonarRawPacketMessage, queue_size=20
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self._cloud_cb, queue_size=5
        )
        rospy.loginfo(
            "multibeam_raw 83P input=%s raw=%s frame=%s beams=%d range=[%.2f, %.2f]",
            self.input_topic,
            self.raw_topic,
            self.frame_id,
            self.beam_count,
            self.min_range_m,
            self.max_range_m,
        )

    def _cloud_cb(self, cloud: PointCloud2) -> None:
        if rospy.is_shutdown():
            return
        source_frame = str(cloud.header.frame_id or "").lstrip("/")
        expected_frame = self.frame_id.lstrip("/")
        if source_frame != expected_frame:
            rospy.logwarn_throttle(
                5.0,
                "multibeam_raw dropped profile: source frame '%s' != '%s'",
                source_frame or "(empty)",
                expected_frame,
            )
            return
        receipt_stamp = rospy.Time.now()
        measurement_stamp = cloud.header.stamp
        if measurement_stamp == rospy.Time():
            measurement_stamp = receipt_stamp
        age_sec = max(0.0, (receipt_stamp - measurement_stamp).to_sec())
        age_units = int(round(age_sec / LATENCY_UNIT_SEC))
        data_latency_units = age_units + self.ping_latency_units
        if data_latency_units > 0xFFFF:
            rospy.logwarn_throttle(
                5.0,
                "multibeam_raw dropped profile: %.6f s latency exceeds 83P field",
                age_sec,
            )
            return

        points = point_records_from_cloud(
            cloud,
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
        )
        ranges, intensities = beam_profile_from_points(
            points,
            beam_count=self.beam_count,
            start_angle_deg=self.start_angle_deg,
            sector_size_deg=self.sector_size_deg,
            angle_increment_deg=self.angle_increment_deg,
            range_resolution_mm=self.range_resolution_mm,
            sound_speed_m_s=self.sound_speed_m_s,
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
        )
        interrogation_time_sec = measurement_stamp.to_sec() - (
            self.ping_latency_units * LATENCY_UNIT_SEC
        )
        packet = encode_profile_packet(
            ranges,
            intensities,
            interrogation_time_sec=interrogation_time_sec,
            samples_per_beam=self.samples_per_beam,
            sector_size_deg=self.sector_size_deg,
            start_angle_deg=self.start_angle_deg,
            angle_increment_deg=self.angle_increment_deg,
            acoustic_range_m=self.max_range_m,
            acoustic_frequency_khz=self.acoustic_frequency_khz,
            sound_speed_m_s=self.sound_speed_m_s,
            range_resolution_mm=self.range_resolution_mm,
            ping_number=self.sequence,
            ping_latency_units=self.ping_latency_units,
            data_latency_units=data_latency_units,
            center_ping_offset_units=self.center_ping_offset_units,
            include_intensity=self.include_intensity,
        )
        msg = SonarRawPacketMessage()
        msg.header.seq = self.sequence & 0xFFFFFFFF
        msg.header.stamp = receipt_stamp
        msg.header.frame_id = self.frame_id
        msg.provider = self.provider
        msg.model = self.model
        msg.packet_kind = PACKET_KIND.decode("ascii")
        msg.source_endpoint = self.source_endpoint
        msg.extrinsic_revision = self.extrinsic_revision
        msg.sequence = self.sequence
        msg.payload = list(packet)
        self.sequence += 1
        try:
            self.publisher.publish(msg)
        except ROSException as exc:
            if rospy.is_shutdown() or "closed topic" in str(exc):
                return
            raise
        rospy.logdebug(
            "multibeam_raw 83P returns=%d beams=%d bytes=%d latency_units=%d",
            sum(1 for sample in ranges if sample),
            self.beam_count,
            len(packet),
            data_latency_units,
        )


def main() -> None:
    rospy.init_node("multibeam_raw")
    MultibeamRawNode()
    rospy.spin()


if __name__ == "__main__":
    main()
