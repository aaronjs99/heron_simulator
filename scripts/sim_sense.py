#!/usr/bin/env python3
"""Publish an explicitly synthetic Heron ``/sense`` contract for simulation.

The real MCU owns this topic on hardware.  Gazebo has no battery monitor or
motor-current sensor, so this bridge deliberately publishes only stable,
non-calibration telemetry required by the shared runtime contract.  Its
current fields remain zero and must never be used for actuator identification.
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
                        "current_semantics": "synthetic_zero",
                    },
                    sort_keys=True,
                )
            )
        )
        rospy.loginfo(
            "sim_sense topic=%s rate=%.1fHz battery=%.2fV " "currents=synthetic_zero",
            self.topic,
            self.rate_hz,
            self.battery_v,
        )

    def spin(self) -> None:
        period_sec = 1.0 / self.rate_hz
        while not rospy.is_shutdown():
            message = Sense()
            message.header.stamp = rospy.Time.now()
            message.battery = self.battery_v
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
