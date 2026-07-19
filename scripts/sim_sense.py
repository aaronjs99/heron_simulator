#!/usr/bin/env python3
"""Publish an explicitly synthetic Heron ``/sense`` contract for simulation.

The real MCU owns this topic on hardware.  Gazebo has no battery monitor or
motor-current sensor. When the empirical actuator proxy is active, this bridge
reports its current-shaped state while retaining non-physical,
non-calibration provenance.
"""

from __future__ import annotations

import time

import rospy
from heron_msgs.msg import Sense
from std_msgs.msg import String
import json


class SimSense:
    """Provide fresh simulated MCU telemetry without claiming physical truth."""

    def __init__(self) -> None:
        rospy.init_node("sim_sense")
        self.topic = str(rospy.get_param("~topic", "/sense"))
        self.rate_hz = max(0.1, float(rospy.get_param("~rate_hz", 10.0)))
        self.battery_v = float(rospy.get_param("~battery_v", 16.0))
        self.actuator_state_topic = str(
            rospy.get_param(
                "~actuator_state_topic",
                "/cmd_drive_to_thrusters/actuator_state",
            )
        )
        self.actuator_state_timeout_sec = max(
            0.0, float(rospy.get_param("~actuator_state_timeout_sec", 0.5))
        )
        self.current_left_a = 0.0
        self.current_right_a = 0.0
        self.actuator_state_receipt_sec = -float("inf")
        self.publisher = rospy.Publisher(self.topic, Sense, queue_size=10)
        self.source_status_publisher = rospy.Publisher(
            "~source_status", String, queue_size=1, latch=True
        )
        self.source_status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "source": "synthetic_simulation",
                        "calibration_eligible": False,
                        "physical_telemetry": False,
                        "current_semantics": "real_curve_current_proxy_when_fresh",
                        "actuator_state_topic": self.actuator_state_topic,
                    },
                    sort_keys=True,
                )
            )
        )
        rospy.loginfo(
            "sim_sense topic=%s rate=%.1fHz battery=%.2fV current_source=%s",
            self.topic,
            self.rate_hz,
            self.battery_v,
            self.actuator_state_topic,
        )
        rospy.Subscriber(
            self.actuator_state_topic,
            String,
            self._actuator_state_cb,
            queue_size=10,
        )

    def _actuator_state_cb(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if payload.get("source") != "synthetic_simulation":
                return
            if payload.get("calibration_eligible") is not False:
                return
            self.current_left_a = max(0.0, float(payload["left_current_a"]))
            self.current_right_a = max(0.0, float(payload["right_current_a"]))
            self.actuator_state_receipt_sec = rospy.Time.now().to_sec()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            rospy.logwarn_throttle(5.0, "sim_sense rejected malformed actuator state")

    def spin(self) -> None:
        period_sec = 1.0 / self.rate_hz
        while not rospy.is_shutdown():
            message = Sense()
            message.header.stamp = rospy.Time.now()
            message.battery = self.battery_v
            state_age = message.header.stamp.to_sec() - self.actuator_state_receipt_sec
            if 0.0 <= state_age <= self.actuator_state_timeout_sec:
                message.current_left = self.current_left_a
                message.current_right = self.current_right_a
            else:
                message.current_left = 0.0
                message.current_right = 0.0
            message.rc = 0
            message.rc_throttle = 0
            message.rc_rotation = 0
            message.rc_enable = 0
            self.publisher.publish(message)
            # Gazebo publishes /clock after this node starts. Wall sleep keeps
            # the contract live during that brief bootstrap without warnings.
            time.sleep(period_sec)


if __name__ == "__main__":
    try:
        SimSense().spin()
    except rospy.ROSInterruptException:
        pass
