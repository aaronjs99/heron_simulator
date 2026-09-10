#!/usr/bin/env python3
"""Publish explicitly synthetic Heron ``/sense`` and ``/status`` contracts.

The real MCU owns this topic on hardware. Gazebo has no battery monitor or
motor-current sensor, so this bridge reports the canonical simulator plant's
synthetic actuator state with explicit non-physical, non-calibration provenance.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from heron_msgs.msg import Sense, Status
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class SimSense(Node):
    """Provide fresh simulated MCU telemetry without claiming physical truth."""

    def __init__(self) -> None:
        super().__init__("sim_sense")
        self.topic = str(self.declare_parameter("topic", "/sense").value)
        self.status_topic = str(self.declare_parameter("status_topic", "/status").value)
        self.rate_hz = max(0.1, float(self.declare_parameter("rate_hz", 10.0).value))
        self.status_rate_hz = max(
            0.1, float(self.declare_parameter("status_rate_hz", 1.0).value)
        )
        self.battery_v = float(self.declare_parameter("battery_v", 16.0).value)
        self.vehicle_battery_fraction = float(
            self.declare_parameter("vehicle_battery_fraction", 1.0).value
        )
        self.payload_battery_fraction = float(
            self.declare_parameter("payload_battery_fraction", 1.0).value
        )
        self.actuator_state_topic = str(
            self.declare_parameter(
                "actuator_state_topic",
                "/cmd_drive_to_thrusters/actuator_state",
            ).value
        )
        self.actuator_state_timeout_sec = max(
            0.0, float(self.declare_parameter("actuator_state_timeout_sec", 0.5).value)
        )
        self.current_left_a = 0.0
        self.current_right_a = 0.0
        self.actuator_state_receipt_sec = -float("inf")
        self.started_ros_sec = None
        self.last_status_ros_sec = None
        self.next_status_wall_sec = time.monotonic()
        self.motor_power_consumed_wh = 0.0
        self.publisher = self.create_publisher(Sense, self.topic, 10)
        self.status_publisher = self.create_publisher(Status, self.status_topic, 2)
        self.vehicle_battery_publisher = self.create_publisher(
            BatteryState,
            str(self.declare_parameter("vehicle_battery_topic", "/battery/heron_state").value),
            2,
        )
        self.payload_battery_publisher = self.create_publisher(
            BatteryState,
            str(self.declare_parameter("payload_battery_topic", "/sense_ighandle").value),
            2,
        )
        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.source_status_publisher = self.create_publisher(
            String, "~/source_status", latched_qos
        )
        self.source_status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "source": "synthetic_simulation",
                        "calibration_eligible": False,
                        "physical_telemetry": False,
                        "current_semantics": "synthetic_battery_side_current_when_fresh",
                        "actuator_state_topic": self.actuator_state_topic,
                    },
                    sort_keys=True,
                )
            )
        )
        self.get_logger().info(
            "sim_sense topic={} rate={:.1f}Hz status_topic={} status_rate={:.1f}Hz "
            "battery={:.2f}V current_source={}".format(
                self.topic,
                self.rate_hz,
                self.status_topic,
                self.status_rate_hz,
                self.battery_v,
                self.actuator_state_topic,
            )
        )
        self.create_subscription(
            String,
            self.actuator_state_topic,
            self._actuator_state_cb,
            10,
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _battery_state(self, stamp, fraction: float, location: str) -> BatteryState:
        """Build explicitly synthetic, fresh battery telemetry for simulation."""
        state = BatteryState()
        state.header.stamp = stamp
        state.voltage = self.battery_v
        state.percentage = min(1.0, max(0.0, float(fraction)))
        state.present = True
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        state.location = location
        state.serial_number = "synthetic_simulation"
        return state

    def _actuator_state_cb(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if payload.get("source") != "synthetic_simulation":
                return
            if payload.get("calibration_eligible") is not False:
                return
            self.current_left_a = max(0.0, float(payload["left_current_a"]))
            self.current_right_a = max(0.0, float(payload["right_current_a"]))
            self.actuator_state_receipt_sec = self.get_clock().now().nanoseconds / 1e9
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn(
                "sim_sense rejected malformed actuator state", throttle_duration_sec=5.0
            )

    def _tick(self) -> None:
        message = Sense()
        message.header.stamp = self.get_clock().now().to_msg()
        message.battery = self.battery_v
        stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        state_age = stamp_sec - self.actuator_state_receipt_sec
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
        self.vehicle_battery_publisher.publish(
            self._battery_state(
                message.header.stamp,
                self.vehicle_battery_fraction,
                "simulated_vehicle",
            )
        )
        self.payload_battery_publisher.publish(
            self._battery_state(
                message.header.stamp,
                self.payload_battery_fraction,
                "simulated_payload",
            )
        )
        now_wall_sec = time.monotonic()
        if now_wall_sec >= self.next_status_wall_sec:
            if self.started_ros_sec is None:
                self.started_ros_sec = stamp_sec
            if self.last_status_ros_sec is not None:
                elapsed_sec = max(0.0, stamp_sec - self.last_status_ros_sec)
                self.motor_power_consumed_wh += (
                    self.battery_v
                    * (message.current_left + message.current_right)
                    * elapsed_sec
                    / 3600.0
                )
            self.last_status_ros_sec = stamp_sec
            status = Status()
            status.header.stamp = message.header.stamp
            status.hardware_id = "synthetic_simulation"
            uptime_sec = max(0.0, stamp_sec - self.started_ros_sec)
            status.mcu_uptime = Duration(seconds=uptime_sec).to_msg()
            status.connection_uptime = Duration(seconds=uptime_sec).to_msg()
            status.pcb_temperature = 0.0
            status.user_current = message.current_left + message.current_right
            status.user_power_consumed = 0.0
            status.motor_power_consumed = self.motor_power_consumed_wh
            status.total_power_consumed = self.motor_power_consumed_wh
            self.status_publisher.publish(status)
            self.next_status_wall_sec = now_wall_sec + 1.0 / self.status_rate_hz


def main() -> None:
    rclpy.init()
    node = SimSense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()