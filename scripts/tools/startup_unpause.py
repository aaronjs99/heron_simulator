#!/usr/bin/env python3
"""Unpause Gazebo after a short wall-clock bootstrap delay."""

import time

import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node("startup_unpause", anonymous=False)

    delay_sec = max(0.0, float(rospy.get_param("~delay_sec", 10.0)))
    service_name = rospy.get_param("~service", "/gazebo/unpause_physics")

    rospy.loginfo(
        "startup_unpause: waiting %.1fs before calling %s", delay_sec, service_name
    )
    time.sleep(delay_sec)

    if rospy.is_shutdown():
        return

    try:
        rospy.wait_for_service(service_name, timeout=15.0)
        unpause = rospy.ServiceProxy(service_name, Empty)
        unpause()
        rospy.loginfo("startup_unpause: unpaused Gazebo via %s", service_name)
    except Exception as exc:
        rospy.logwarn("startup_unpause: failed to unpause Gazebo: %s", exc)


if __name__ == "__main__":
    main()
