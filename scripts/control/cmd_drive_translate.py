#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Guy Stoppi <gstoppi@clearpathrobotics.com>
# @copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from geometry_msgs.msg import Wrench
from heron_msgs.msg import Drive
import rospy
import numpy as np


class ThrusterTranslator:
    """
    Translates normalized drive commands to physical thrust forces (Newtons)
    using a linear interpolation model derived from empirical vessel trials.
    This implementation maps the control input directly to Newtonian forces
    applied to the vehicle's propulsion links within the Gazebo environment.
    """

    def __init__(self):
        rospy.init_node("cmd_drive_to_thrusters")

        namespace = rospy.get_param("~namespace", "")

        # Thrust model parameters (from empirical data)
        self.input_points = np.array(
            [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        )
        self.output_thrust = np.array(
            [-19.88, -16.52, -12.6, -5.6, -1.4, 0.0, 2.24, 9.52, 21.28, 28.0, 33.6]
        )

        if namespace:
            prefix = f"/{namespace}"
        else:
            prefix = ""

        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.75))
        self.time_constant_up = float(rospy.get_param("~time_constant_up", 0.35))
        self.time_constant_down = float(rospy.get_param("~time_constant_down", 0.25))
        self.max_delta_per_sec = float(rospy.get_param("~max_delta_per_sec", 2.5))
        self.command_deadband = float(rospy.get_param("~command_deadband", 0.03))
        self.left_scale = float(rospy.get_param("~left_scale", 1.0))
        self.right_scale = float(rospy.get_param("~right_scale", 1.0))
        self.left_bias = float(rospy.get_param("~left_bias", 0.0))
        self.right_bias = float(rospy.get_param("~right_bias", 0.0))
        self.thrust_noise_stddev = float(rospy.get_param("~thrust_noise_stddev", 0.0))
        self.rng = np.random.default_rng(
            int(rospy.get_param("~random_seed", rospy.Time.now().to_nsec() % 2**32))
        )

        default_left_topic = f"{prefix}/thrusters/1/input" if prefix else "/thrusters/1/input"
        default_right_topic = f"{prefix}/thrusters/0/input" if prefix else "/thrusters/0/input"
        default_drive_topic = "cmd_drive"

        left_topic = rospy.get_param("~left_thruster_topic", default_left_topic)
        right_topic = rospy.get_param("~right_thruster_topic", default_right_topic)
        drive_topic = rospy.get_param("~drive_topic", default_drive_topic)

        self.p_left = rospy.Publisher(left_topic, Wrench, queue_size=1)
        self.p_right = rospy.Publisher(right_topic, Wrench, queue_size=1)

        self.sub = rospy.Subscriber(drive_topic, Drive, self.callback)
        self.target_left = 0.0
        self.target_right = 0.0
        self.actual_left = 0.0
        self.actual_right = 0.0
        self.last_cmd_time = rospy.Time.now()
        self.last_update_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.update)
        rospy.loginfo(
            "Thruster translator initialized for namespace: %s drive=%s left=%s right=%s",
            namespace,
            drive_topic,
            left_topic,
            right_topic,
        )

    def get_thrust(self, cmd):
        """Linearly interpolate thrust force from normalized command [-1, 1]."""
        return np.interp(cmd, self.input_points, self.output_thrust)

    def callback(self, msg):
        self.target_left = self.shape_command(msg.left, self.left_scale, self.left_bias)
        self.target_right = self.shape_command(
            msg.right, self.right_scale, self.right_bias
        )
        self.last_cmd_time = rospy.Time.now()

    def shape_command(self, cmd, scale, bias):
        cmd = float(np.clip(cmd, -1.0, 1.0))
        if abs(cmd) < self.command_deadband:
            return 0.0
        shaped = (cmd * scale) + (bias if cmd > 0.0 else -bias)
        return float(np.clip(shaped, -1.0, 1.0))

    def slew_and_lag(self, actual, target, dt):
        max_step = self.max_delta_per_sec * dt
        slewed_target = actual + np.clip(target - actual, -max_step, max_step)
        tau = self.time_constant_up if abs(slewed_target) > abs(actual) else self.time_constant_down
        alpha = 1.0 if tau <= 1e-6 else 1.0 - np.exp(-dt / tau)
        return actual + alpha * (slewed_target - actual)

    def update(self, _event):
        now = rospy.Time.now()
        dt = max((now - self.last_update_time).to_sec(), 1e-3)
        self.last_update_time = now

        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.target_left = 0.0
            self.target_right = 0.0

        self.actual_left = self.slew_and_lag(self.actual_left, self.target_left, dt)
        self.actual_right = self.slew_and_lag(
            self.actual_right, self.target_right, dt
        )

        left_force = self.get_thrust(self.actual_left)
        right_force = self.get_thrust(self.actual_right)

        if self.thrust_noise_stddev > 0.0:
            left_force += self.rng.normal(0.0, self.thrust_noise_stddev)
            right_force += self.rng.normal(0.0, self.thrust_noise_stddev)

        left_wrench = Wrench()
        left_wrench.force.x = left_force
        self.p_left.publish(left_wrench)

        right_wrench = Wrench()
        right_wrench.force.x = right_force
        self.p_right.publish(right_wrench)


if __name__ == "__main__":
    try:
        node = ThrusterTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
