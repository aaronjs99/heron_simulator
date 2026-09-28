#!/usr/bin/env python3
"""Publish explicitly simulated acquisition records on the shared sensor contract."""

from __future__ import annotations

import copy
import threading
from pathlib import Path
from time import monotonic_ns
from typing import Dict, List, Optional, Set, Tuple
from uuid import uuid4

import rospy
import yaml
from rospy.exceptions import ROSException
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, TimeReference
from ig_handle.msg import AcquisitionTimingEvent, ClockedImu
from spinnaker_camera_driver.msg import CameraFrameCapture, CameraFrameTiming
from wfov_camera_msgs.msg import WFOVImage


def _stamp_ns(stamp: rospy.Time) -> int:
    return int(stamp.to_nsec()) if stamp is not None else 0


def _time_from_ns(value: int) -> rospy.Time:
    value = max(0, int(value))
    return rospy.Time(value // 1_000_000_000, value % 1_000_000_000)


def _sensor_streams(path: str) -> Tuple[List[dict], List[dict]]:
    """Resolve simulator streams from the deployed IGHandle sensor contract."""
    if not path:
        raise ValueError(
            "~sensor_contract_file is required for simulated sensor identity"
        )
    with open(path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    contract = document.get("sensor_contract", document)
    sensors = contract.get("sensors", {}) or {}
    bindings = contract.get("deployment_bindings", {}) or {}
    cameras = []
    lidars = []
    for binding_name, binding in sorted(bindings.items()):
        if not str(binding_name).startswith("inspection_"):
            continue
        sensor_id = str(binding.get("sensor", ""))
        sensor = sensors.get(sensor_id, {}) or {}
        topics = sensor.get("topics", {}) or {}
        required = ("image_raw", "frame_capture", "frame_timing")
        if any(not topics.get(name) for name in required):
            rospy.logwarn(
                "sim timing skips camera binding %s: incomplete sensor contract",
                binding_name,
            )
            continue
        cameras.append(
            {
                "identity": str(binding_name),
                "serial": "sim-" + str(binding_name),
                "image_topic": str(topics["image_raw"]),
                "info_topic": str(topics.get("camera_info", "")),
                "capture_topic": str(topics["frame_capture"]),
                "frame_timing_topic": str(topics["frame_timing"]),
            }
        )
    for sensor_id, sensor in sorted(sensors.items(), key=lambda item: str(item[0])):
        if str(sensor.get("family", "")).strip().lower() != "lidar":
            continue
        points = str((sensor.get("topics", {}) or {}).get("points", "") or "").strip()
        if points:
            identity = str(sensor.get("frame", sensor_id) or sensor_id)
            lidars.append({"identity": identity, "topic": points})
    return cameras, lidars


class SimIgTimingBridge:
    """Expose simulation-clock measurements without manufacturing physical timing."""

    def __init__(self) -> None:
        self.pps_topic = rospy.get_param("~pps_time_topic", "/sensors/pps/time")
        self.camera_time_topic = rospy.get_param(
            "~camera_time_topic", "/sensors/camera/time"
        )
        self.imu_sample_time_topic = rospy.get_param(
            "~imu_sample_time_topic", "/sensors/imu/sample_time"
        )
        self.imu_topic = rospy.get_param("~imu_topic", "/sensors/imu/data")
        self.timed_imu_topic = rospy.get_param(
            "~timed_imu_topic", "/sensors/imu/data_timed"
        )
        self.events_topic = rospy.get_param(
            "~timing_events_topic", "/sensors/timing/events"
        )
        self.sensor_contract_file = str(
            rospy.get_param("~sensor_contract_file", "") or ""
        ).strip()
        self.pps_rate_hz = float(rospy.get_param("~pps_rate_hz", 1.0))
        if self.pps_rate_hz <= 0.0:
            raise ValueError("~pps_rate_hz must be positive")

        self.bridge_id = uuid4().hex
        try:
            self.receipt_host_boot_id = (
                Path("/proc/sys/kernel/random/boot_id").read_text().strip()
            )
        except OSError:
            self.receipt_host_boot_id = ""
        self._lock = threading.RLock()
        self._event_sequence = 0
        self._ros_time_epoch = 0
        self._mapping_revision = 1
        self._last_ros_now_ns: Optional[int] = None
        self._order_state: Dict[str, Tuple[int, int, int]] = {}

        self.pps_sequence = 0
        self.imu_sequence = 0
        self.imu_source_epoch = 0
        self.imu_source_instance_id = "sim:" + self.bridge_id + ":imu"
        self.imu_source_boot_id = "sim:" + self.bridge_id
        self.imu_source_instance_generation = 1

        cameras, lidars = _sensor_streams(self.sensor_contract_file)
        self.camera_streams = cameras
        self.lidar_streams = lidars
        self.camera_info: Dict[str, CameraInfo] = {}
        self.camera_frame_counters = {item["identity"]: 0 for item in cameras}
        self.camera_source_keys: Dict[str, Set[Tuple[int, int, int]]] = {
            item["identity"]: set() for item in cameras
        }
        self.camera_stream_ids = {
            item["identity"]: "sim:{}:{}".format(self.bridge_id, item["identity"])
            for item in cameras
        }

        self.pps_pub = rospy.Publisher(self.pps_topic, TimeReference, queue_size=10)
        self.camera_pub = rospy.Publisher(
            self.camera_time_topic, TimeReference, queue_size=20
        )
        self.imu_pub = rospy.Publisher(
            self.imu_sample_time_topic, TimeReference, queue_size=50
        )
        self.timed_imu_pub = rospy.Publisher(
            self.timed_imu_topic, ClockedImu, queue_size=50
        )
        self.events_pub = rospy.Publisher(
            self.events_topic, AcquisitionTimingEvent, queue_size=1000
        )
        self.camera_capture_pubs = {
            item["identity"]: rospy.Publisher(
                item["capture_topic"], CameraFrameCapture, queue_size=4
            )
            for item in cameras
        }
        self.camera_timing_pubs = {
            item["identity"]: rospy.Publisher(
                item["frame_timing_topic"], CameraFrameTiming, queue_size=16
            )
            for item in cameras
        }
        self.camera_subscribers = []
        for stream in cameras:
            identity = stream["identity"]
            if stream["info_topic"]:
                self.camera_subscribers.append(
                    rospy.Subscriber(
                        stream["info_topic"],
                        CameraInfo,
                        self._camera_info_cb,
                        callback_args=identity,
                        queue_size=1,
                    )
                )
            self.camera_subscribers.append(
                rospy.Subscriber(
                    stream["image_topic"],
                    Image,
                    self._camera_cb,
                    callback_args=stream,
                    queue_size=2,
                )
            )
        self.lidar_subscribers = [
            rospy.Subscriber(
                stream["topic"],
                PointCloud2,
                self._lidar_cb,
                callback_args=stream,
                queue_size=10,
            )
            for stream in lidars
        ]
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self._imu_cb, queue_size=20
        )
        self.pps_timer = rospy.Timer(
            rospy.Duration(1.0 / self.pps_rate_hz), self._pps_cb
        )
        rospy.loginfo(
            "sim_ig_timing contract=%s event_topic=%s cameras=%s lidars=%s",
            self.sensor_contract_file,
            self.events_topic,
            ",".join(item["identity"] for item in cameras) or "<none>",
            ",".join(item["identity"] for item in lidars) or "<none>",
        )

    def _observe_ros_clock(self) -> Tuple[rospy.Time, int, int, int]:
        # Observe receipt time while serialized; callback arrival order is not a clock reset.
        with self._lock:
            now = rospy.Time.now()
            receipt_monotonic_ns = monotonic_ns()
            now_ns = _stamp_ns(now)
            if self._last_ros_now_ns is not None and now_ns < self._last_ros_now_ns:
                self._ros_time_epoch += 1
                self._mapping_revision += 1
                for source_keys in self.camera_source_keys.values():
                    source_keys.clear()
                rospy.logwarn(
                    "sim_ig_timing ROS clock reset epoch=%d revision=%d",
                    self._ros_time_epoch,
                    self._mapping_revision,
                )
            self._last_ros_now_ns = now_ns
            return (
                now,
                receipt_monotonic_ns,
                self._ros_time_epoch,
                self._mapping_revision,
            )

    def _next_event_id(self) -> Tuple[str, int]:
        with self._lock:
            self._event_sequence += 1
            sequence = self._event_sequence
        return "{}:{}".format(self.bridge_id, sequence), sequence

    def _order(
        self, key: str, sequence: int, raw_ns: int, bits: int, clock_epoch: int
    ) -> Tuple[int, int]:
        mask = (1 << bits) - 1
        half = 1 << (bits - 1)
        with self._lock:
            previous = self._order_state.get(key)
            if previous is None:
                result = AcquisitionTimingEvent.ORDER_FIRST
                missing = 0
            else:
                previous_sequence, previous_raw_ns, previous_epoch = previous
                if clock_epoch != previous_epoch:
                    result = AcquisitionTimingEvent.ORDER_CLOCK_EPOCH
                    missing = 0
                else:
                    delta = (int(sequence) - previous_sequence) & mask
                    if delta == 0:
                        result = AcquisitionTimingEvent.ORDER_DUPLICATE
                        missing = 0
                    elif delta >= half:
                        result = AcquisitionTimingEvent.ORDER_OUT_OF_ORDER
                        missing = 0
                    elif raw_ns and previous_raw_ns and raw_ns < previous_raw_ns:
                        result = AcquisitionTimingEvent.ORDER_OUT_OF_ORDER
                        missing = 0
                    elif delta == 1:
                        result = AcquisitionTimingEvent.ORDER_FORWARD
                        missing = 0
                    else:
                        result = AcquisitionTimingEvent.ORDER_GAP
                        missing = delta - 1
            if result not in (
                AcquisitionTimingEvent.ORDER_DUPLICATE,
                AcquisitionTimingEvent.ORDER_OUT_OF_ORDER,
            ):
                self._order_state[key] = (int(sequence), int(raw_ns), int(clock_epoch))
            return result, missing

    def _event(
        self,
        kind: int,
        source_key: str,
        source_id: str,
        source_instance_id: str,
        source_sequence: int,
        sequence_bits: int,
        sequence_origin: int,
        raw_stamp: rospy.Time,
        receipt: rospy.Time,
        receipt_monotonic_ns: int,
        clock_epoch: int,
        mapping_revision: int,
        sensor_frame_counter: Optional[int] = None,
    ) -> AcquisitionTimingEvent:
        event_id, bridge_sequence = self._next_event_id()
        raw_ns = _stamp_ns(raw_stamp)
        valid = raw_ns > 0
        order, missing = self._order(
            source_id + "|" + source_instance_id,
            source_sequence,
            raw_ns,
            sequence_bits,
            clock_epoch,
        )
        event = AcquisitionTimingEvent()
        event.header.stamp = receipt
        event.header.frame_id = source_key
        event.kind = kind
        event.identity_scope = AcquisitionTimingEvent.IDENTITY_SCOPE_BRIDGE_INSTANCE
        event.event_id = event_id
        event.bridge_event_sequence = bridge_sequence
        event.source_key = source_key
        event.source_id = source_id
        event.source_instance_id = source_instance_id
        event.source_sequence_valid = True
        event.source_sequence = int(source_sequence)
        event.source_sequence_origin = sequence_origin
        event.bridge_instance_id = self.bridge_id
        event.source_boot_id_valid = False
        event.source_clock_epoch = int(clock_epoch)
        event.source_clock_epoch_valid = True
        event.source_sequence_bits = int(sequence_bits)
        event.source_clock_domain = "ros_sim_time"
        event.clock_mapping_source_instance_id = source_instance_id
        event.clock_mapping_source_epoch_id = "ros_sim_time:{}".format(clock_epoch)
        event.raw_source_time_valid = valid
        event.raw_source_time = raw_stamp if valid else rospy.Time()
        event.raw_source_time_seconds = raw_stamp.to_sec() if valid else 0.0
        event.receipt_ros_time = receipt
        event.receipt_monotonic_ns = int(receipt_monotonic_ns)
        event.receipt_host_boot_id = self.receipt_host_boot_id
        event.ros_time_epoch = int(clock_epoch)
        event.order_disposition = order
        event.missing_source_sequences = int(missing)
        event.correlation_status = AcquisitionTimingEvent.CORRELATION_UNRESOLVED
        event.observed_source_generation = 0
        event.source_generation_inferred = False
        event.mapped_continuity_epoch = int(clock_epoch)
        event.mapped_acquisition_time_valid = valid
        event.mapped_acquisition_time = raw_stamp if valid else rospy.Time()
        event.clock_mapping_revision = int(mapping_revision)
        event.timing_uncertainty_sec = 0.0
        event.clock_mapping_calibrated = valid
        if sensor_frame_counter is not None:
            event.sensor_frame_counter_valid = True
            event.sensor_frame_counter = int(sensor_frame_counter)
        return event

    def _time_reference(
        self, stamp: rospy.Time, frame_id: str, source: str, receipt: rospy.Time
    ) -> TimeReference:
        message = TimeReference()
        message.header.stamp = receipt
        message.header.frame_id = frame_id
        message.time_ref = stamp if _stamp_ns(stamp) else rospy.Time()
        message.source = source
        return message

    @staticmethod
    def _safe_publish(publisher: rospy.Publisher, message) -> None:
        if rospy.is_shutdown():
            return
        try:
            publisher.publish(message)
        except ROSException as exc:
            if rospy.is_shutdown() or "closed topic" in str(exc).lower():
                return
            raise

    def _pps_cb(self, _event) -> None:
        receipt, receipt_monotonic, epoch, revision = self._observe_ros_clock()
        stamp = receipt
        self._safe_publish(
            self.pps_pub,
            self._time_reference(
                stamp, "sim_pps", "heron_simulator:ros_sim_time", receipt
            ),
        )
        event = self._event(
            AcquisitionTimingEvent.KIND_PPS_REFERENCE,
            "sim_pps",
            "heron_simulator/pps",
            "sim:{}:pps".format(self.bridge_id),
            self.pps_sequence,
            64,
            AcquisitionTimingEvent.SEQUENCE_COUNTER_SOURCE,
            stamp,
            receipt,
            receipt_monotonic,
            epoch,
            revision,
        )
        self.pps_sequence += 1
        self._safe_publish(self.events_pub, event)

    def _camera_info_cb(self, message: CameraInfo, identity: str) -> None:
        with self._lock:
            self.camera_info[identity] = copy.deepcopy(message)

    def _camera_cb(self, message: Image, stream: dict) -> None:
        identity = stream["identity"]
        source_stamp = message.header.stamp
        source_ns = _stamp_ns(source_stamp)
        receipt, receipt_monotonic, epoch, revision = self._observe_ros_clock()
        source_key = (epoch, int(message.header.seq), source_ns)
        with self._lock:
            if source_key in self.camera_source_keys[identity]:
                rospy.logwarn_throttle(
                    5.0,
                    "sim_ig_timing duplicate frame camera=%s seq=%d stamp_ns=%d",
                    identity,
                    message.header.seq,
                    source_ns,
                )
                return
            self.camera_source_keys[identity].add(source_key)
            frame_counter = self.camera_frame_counters[identity]
            self.camera_frame_counters[identity] = frame_counter + 1
            stream_id = self.camera_stream_ids[identity]
            camera_info = copy.deepcopy(self.camera_info.get(identity, CameraInfo()))

        frame_id = str(message.header.frame_id or "camera_optical_frame")
        capture = CameraFrameCapture()
        capture.header.stamp = receipt
        capture.header.seq = frame_counter & 0xFFFFFFFF
        capture.header.frame_id = frame_id
        capture.camera_serial = stream["serial"]
        capture.stream_id = stream_id
        capture.frame_counter = frame_counter
        capture.device_timestamp_ns = source_ns
        capture.host_monotonic_ns = receipt_monotonic
        frame = WFOVImage()
        frame.header.stamp = receipt
        frame.header.seq = capture.header.seq
        frame.header.frame_id = frame_id
        frame.time_reference = (
            "ros_sim_time; simulated capture; no physical trigger correlation"
        )
        frame.image = copy.deepcopy(message)
        frame.info = camera_info
        frame.info.header.stamp = receipt
        frame.info.header.frame_id = frame_id
        if not frame.info.width:
            frame.info.width = message.width
        if not frame.info.height:
            frame.info.height = message.height
        capture.frame = frame

        self._safe_publish(
            self.camera_pub,
            self._time_reference(
                source_stamp, frame_id, "heron_simulator:" + identity, receipt
            ),
        )
        event = self._event(
            AcquisitionTimingEvent.KIND_CAMERA_FRAME,
            identity,
            "spinnaker_camera_" + stream["serial"],
            stream_id,
            int(message.header.seq),
            32,
            AcquisitionTimingEvent.SEQUENCE_COUNTER_ROS_HEADER,
            source_stamp,
            receipt,
            receipt_monotonic,
            epoch,
            revision,
            sensor_frame_counter=frame_counter,
        )
        self._safe_publish(self.events_pub, event)

        timing = CameraFrameTiming()
        timing.header = capture.header
        timing.camera_serial = capture.camera_serial
        timing.stream_id = stream_id
        timing.frame_counter = frame_counter
        timing.device_timestamp_ns = source_ns
        timing.host_monotonic_ns = receipt_monotonic
        self._safe_publish(self.camera_timing_pubs[identity], timing)
        self._safe_publish(self.camera_capture_pubs[identity], capture)

    def _lidar_cb(self, message: PointCloud2, stream: dict) -> None:
        receipt, receipt_monotonic, epoch, revision = self._observe_ros_clock()
        source_id = "heron_simulator/lidar/" + stream["identity"]
        source_instance = "sim:{}:lidar:{}".format(self.bridge_id, stream["identity"])
        raw_stamp = message.header.stamp
        event = self._event(
            AcquisitionTimingEvent.KIND_LIDAR_CLOUD_HEADER,
            stream["identity"],
            source_id,
            source_instance,
            int(message.header.seq),
            32,
            AcquisitionTimingEvent.SEQUENCE_COUNTER_ROS_HEADER,
            raw_stamp,
            receipt,
            receipt_monotonic,
            epoch,
            revision,
        )
        self._safe_publish(self.events_pub, event)

    def _imu_cb(self, message: Imu) -> None:
        source_stamp = message.header.stamp
        receipt, receipt_monotonic, epoch, revision = self._observe_ros_clock()
        frame_id = str(message.header.frame_id or "imu_link")
        self._safe_publish(
            self.imu_pub,
            self._time_reference(
                source_stamp, frame_id, "heron_simulator:ros_sim_time", receipt
            ),
        )

        timed = ClockedImu()
        timed.header.stamp = source_stamp
        timed.header.frame_id = frame_id
        timed.source_id = "heron_simulator/imu"
        timed.source_boot_id = self.imu_source_boot_id
        timed.device_boot_id_valid = False
        timed.device_boot_id = ""
        timed.source_instance_id = self.imu_source_instance_id
        timed.device_host_boot_id = self.receipt_host_boot_id
        timed.source_instance_generation = self.imu_source_instance_generation
        timed.device_clock_epoch = epoch
        timed.clock_domain = "ros_sim_time"
        timed.source_epoch = epoch
        timed.source_sequence = self.imu_sequence
        timed.source_sequence_bits = 64
        timed.source_time = source_stamp
        timed.receipt_time = receipt
        timed.receipt_host_boot_id = self.receipt_host_boot_id
        timed.receipt_monotonic_ns = receipt_monotonic
        timed.ros_time_epoch = epoch
        timed.timing_uncertainty_sec = 0.0
        timed.clock_mapping_revision = revision
        timed.clock_mapping_calibrated = _stamp_ns(source_stamp) > 0
        timed.measurement = copy.deepcopy(message)
        timed.measurement.header.stamp = source_stamp
        timed.measurement.header.frame_id = frame_id
        self.imu_sequence += 1
        self._safe_publish(self.timed_imu_pub, timed)


def main() -> None:
    rospy.init_node("sim_ig_timing")
    SimIgTimingBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
