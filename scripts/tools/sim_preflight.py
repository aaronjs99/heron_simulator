#!/usr/bin/env python3
"""Fail fast when simulation prerequisites are not satisfied."""

import socket
import sys

import rospy
from gazebo_msgs.srv import GetWorldProperties


def port_in_use(port: int, host: str = "127.0.0.1") -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex((host, int(port))) == 0
    finally:
        sock.close()


def main():
    rospy.init_node("simulation_preflight", anonymous=False)

    gazebo_master_port = int(rospy.get_param("~gazebo_master_port", 11345))
    allow_existing_gazebo = bool(rospy.get_param("~allow_existing_gazebo", False))
    required_free_ports = list(rospy.get_param("~required_free_ports", []))

    errors = []

    if not allow_existing_gazebo and port_in_use(gazebo_master_port):
        errors.append(f"Gazebo master port {gazebo_master_port} is already in use.")
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=0.5)
            get_world_properties = rospy.ServiceProxy(
                "/gazebo/get_world_properties", GetWorldProperties
            )
            world = get_world_properties()
            model_names = list(world.model_names)
            if "heron" in model_names:
                errors.append("Existing Gazebo world already contains model 'heron'.")
        except Exception:
            pass

    for port in required_free_ports:
        port = int(port)
        if port_in_use(port):
            errors.append(f"Required TCP port {port} is already in use.")

    if errors:
        for err in errors:
            rospy.logerr("[sim_preflight] %s", err)
        rospy.logerr(
            "[sim_preflight] Clean the old session first, e.g. "
            "`reset_gazebo.sh --full`, then relaunch."
        )
        sys.exit(1)

    rospy.loginfo("[sim_preflight] Environment looks clean.")
    rospy.loginfo("[sim_preflight] Standing by.")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
