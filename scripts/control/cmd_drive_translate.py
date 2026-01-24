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

        namespace = rospy.get_param("~namespace", "heron")

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
            # Match the double slash artifact from xacro if it's happening there
            # snippets.xacro: /${robot_namespace}/thrusters/... -> //thrusters/...
            prefix = "/"

        self.p_left = rospy.Publisher(
            f"{prefix}/thrusters/1/input", Wrench, queue_size=1
        )
        self.p_right = rospy.Publisher(
            f"{prefix}/thrusters/0/input", Wrench, queue_size=1
        )

        self.sub = rospy.Subscriber("cmd_drive", Drive, self.callback)
        rospy.loginfo(f"Thruster translator initialized for namespace: {namespace}")

    def get_thrust(self, cmd):
        """Linearly interpolate thrust force from normalized command [-1, 1]."""
        return np.interp(cmd, self.input_points, self.output_thrust)

    def callback(self, msg):
        # Left Thruster
        left_wrench = Wrench()
        left_wrench.force.x = self.get_thrust(msg.left)
        self.p_left.publish(left_wrench)

        # Right Thruster
        right_wrench = Wrench()
        right_wrench.force.x = self.get_thrust(msg.right)
        self.p_right.publish(right_wrench)


if __name__ == "__main__":
    try:
        node = ThrusterTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
