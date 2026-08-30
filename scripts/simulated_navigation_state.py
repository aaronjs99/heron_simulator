#!/usr/bin/env python3
"""Publish an uncertainty-bearing simulated navigation state.

The clean simulator pose remains an independent evaluation signal. This node
constructs a separate synthetic sensor by adding declared noise and publishing
the matching covariance for closed-loop planner and controller studies.
"""

from __future__ import annotations

import copy
import math
import threading
from collections import deque
from typing import Deque, Tuple

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SimulatedNavigationState:
    """Convert clean simulator state into a reproducible noisy sensor stream."""

    _POSE_INDICES = (0, 1, 5)
    _TWIST_INDICES = (0, 1, 5)

    def __init__(self) -> None:
        runtime_mode = str(rospy.get_param("/grande/runtime/mode", "") or "").strip()
        use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        global_authority = str(
            rospy.get_param("/grande/runtime/global_frame_authority", "") or ""
        ).strip()
        if runtime_mode != "sim" or not use_sim_time or global_authority != "identity":
            raise RuntimeError(
                "simulated navigation state requires sim mode, simulated time, and identity map-to-odom authority"
            )
        self.input_topic = self._required_text("~input_topic")
        self.output_topic = self._required_text("~output_topic")
        self.input_frame = self._required_text("~input_frame")
        self.parent_frame = self._required_text("~parent_frame")
        self.child_frame = self._required_text("~child_frame")
        self.pose_stddev = self._stddevs("~pose_stddev", 3)
        self.twist_stddev = self._stddevs("~twist_stddev", 3)
        self.pose_bias_stddev = self._stddevs("~pose_bias_stddev", 3)
        self.twist_bias_stddev = self._stddevs("~twist_bias_stddev", 3)
        self.frame_stddev = self._stddevs("~frame_stddev", 3)
        self.bias_correlation_time_sec = self._positive("~bias_correlation_time_sec")
        self.latency_sec = self._nonnegative("~latency_sec")
        self.latency_jitter_stddev_sec = self._nonnegative("~latency_jitter_stddev_sec")
        self.latency_jitter_bound_sec = self._nonnegative("~latency_jitter_bound_sec")
        self.dropout_probability = self._nonnegative("~dropout_probability")
        if self.dropout_probability >= 1.0:
            raise RuntimeError("~dropout_probability must be less than one")
        self.random = np.random.RandomState(int(rospy.get_param("~random_seed", 0)))
        self.pose_bias = self.random.normal(0.0, self.pose_bias_stddev)
        self.twist_bias = self.random.normal(0.0, self.twist_bias_stddev)
        self.frame_offset = self.random.normal(0.0, self.frame_stddev)
        self.last_measurement_stamp = None
        self.last_flush_time = None
        self.queue: Deque[Tuple[rospy.Time, Odometry]] = deque()
        self.lock = threading.Lock()
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=50)
        rospy.Subscriber(
            self.input_topic,
            Odometry,
            self._handle_state,
            queue_size=50,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(rospy.Duration(0.01), self._flush, reset=True)
        rospy.loginfo(
            "simulated_navigation_state: input=%s output=%s pose_stddev=%s twist_stddev=%s seed=%d",
            self.input_topic,
            self.output_topic,
            self.pose_stddev.tolist(),
            self.twist_stddev.tolist(),
            int(rospy.get_param("~random_seed", 0)),
        )

    @staticmethod
    def _required_text(name: str) -> str:
        value = str(rospy.get_param(name, "") or "").strip()
        if not value:
            raise RuntimeError(f"{name} is required")
        return value

    @staticmethod
    def _stddevs(name: str, size: int) -> np.ndarray:
        values = np.asarray(rospy.get_param(name, []), dtype=float)
        if values.shape != (size,) or not np.isfinite(values).all():
            raise RuntimeError(f"{name} must contain {size} finite values")
        if np.any(values <= 0.0):
            raise RuntimeError(f"{name} values must be positive")
        return values

    @staticmethod
    def _positive(name: str) -> float:
        value = float(rospy.get_param(name, float("nan")))
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{name} must be positive and finite")
        return value

    @staticmethod
    def _nonnegative(name: str) -> float:
        value = float(rospy.get_param(name, float("nan")))
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"{name} must be nonnegative and finite")
        return value

    @staticmethod
    def _covariance(indices, stddev: np.ndarray) -> list:
        covariance = np.zeros((6, 6), dtype=float)
        for index, sigma in zip(indices, stddev):
            covariance[index, index] = float(sigma * sigma)
        # Unused vertical and roll/pitch axes are intentionally assigned large,
        # finite uncertainty rather than presented as exact measurements.
        for index in set(range(6)) - set(indices):
            covariance[index, index] = 1.0
        return covariance.reshape(-1).tolist()

    def _advance_bias(self, stamp: rospy.Time) -> None:
        if self.last_measurement_stamp is None:
            self.last_measurement_stamp = stamp
            return
        dt = float((stamp - self.last_measurement_stamp).to_sec())
        self.last_measurement_stamp = stamp
        if dt <= 0.0 or not math.isfinite(dt):
            return
        correlation = math.exp(-dt / self.bias_correlation_time_sec)
        innovation = math.sqrt(max(0.0, 1.0 - correlation * correlation))
        self.pose_bias = correlation * self.pose_bias + innovation * self.random.normal(
            0.0, self.pose_bias_stddev
        )
        self.twist_bias = (
            correlation * self.twist_bias
            + innovation * self.random.normal(0.0, self.twist_bias_stddev)
        )

    def _handle_state(self, source: Odometry) -> None:
        if source.header.stamp == rospy.Time(0):
            rospy.logerr_throttle(
                2.0, "simulated_navigation_state rejected zero measurement stamp"
            )
            return
        if str(source.header.frame_id).lstrip("/") != self.input_frame.lstrip("/"):
            rospy.logerr_throttle(
                2.0, "simulated_navigation_state rejected unexpected input frame"
            )
            return
        self._advance_bias(source.header.stamp)
        if self.random.uniform() < self.dropout_probability:
            return
        state = Odometry()
        state.header.stamp = source.header.stamp
        state.header.frame_id = self.parent_frame
        state.child_frame_id = self.child_frame
        state.pose.pose = copy.deepcopy(source.pose.pose)
        state.twist.twist = copy.deepcopy(source.twist.twist)

        pose_noise = (
            self.random.normal(0.0, self.pose_stddev)
            + self.pose_bias
            + self.frame_offset
        )
        state.pose.pose.position.x += float(pose_noise[0])
        state.pose.pose.position.y += float(pose_noise[1])
        quaternion = state.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        noisy_quaternion = quaternion_from_euler(roll, pitch, yaw + pose_noise[2])
        (
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w,
        ) = noisy_quaternion

        twist_noise = self.random.normal(0.0, self.twist_stddev) + self.twist_bias
        state.twist.twist.linear.x += float(twist_noise[0])
        state.twist.twist.linear.y += float(twist_noise[1])
        state.twist.twist.angular.z += float(twist_noise[2])
        pose_total_stddev = np.sqrt(
            self.pose_stddev**2 + self.pose_bias_stddev**2 + self.frame_stddev**2
        )
        twist_total_stddev = np.sqrt(self.twist_stddev**2 + self.twist_bias_stddev**2)
        state.pose.covariance = self._covariance(self._POSE_INDICES, pose_total_stddev)
        state.twist.covariance = self._covariance(
            self._TWIST_INDICES, twist_total_stddev
        )

        values = np.asarray(
            [
                state.pose.pose.position.x,
                state.pose.pose.position.y,
                state.pose.pose.orientation.z,
                state.pose.pose.orientation.w,
                state.twist.twist.linear.x,
                state.twist.twist.linear.y,
                state.twist.twist.angular.z,
            ]
        )
        if not np.isfinite(values).all() or not math.isfinite(yaw):
            rospy.logerr_throttle(
                2.0, "simulated_navigation_state rejected non-finite source state"
            )
            return
        jitter = float(
            np.clip(
                self.random.normal(0.0, self.latency_jitter_stddev_sec),
                -self.latency_jitter_bound_sec,
                self.latency_jitter_bound_sec,
            )
        )
        delay = max(0.0, self.latency_sec + jitter)
        release = rospy.Time.now() + rospy.Duration(delay)
        with self.lock:
            self.queue.append((release, state))

    def _flush(self, _event) -> None:
        now = rospy.Time.now()
        ready = []
        with self.lock:
            if self.last_flush_time is not None and now < self.last_flush_time:
                self.queue.clear()
            self.last_flush_time = now
            while self.queue and self.queue[0][0] <= now:
                _, state = self.queue.popleft()
                ready.append(state)
        for state in ready:
            self.publisher.publish(state)


if __name__ == "__main__":
    rospy.init_node("simulated_navigation_state")
    SimulatedNavigationState()
    rospy.spin()
