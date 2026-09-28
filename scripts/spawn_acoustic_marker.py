#!/usr/bin/env python3
"""Spawn one descriptor-driven acoustic marker in Gazebo."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from range_aid.marker.model import MarkerInstance, load_instance

from models.acoustic_marker_model import load_descriptor, render_sdf
from models.parameters import strict_bool


def _token(value: object, label: str) -> str:
    token = str(value or "").strip()
    if not token:
        raise ValueError(f"{label} must be non-empty")
    return token


def _pose_from_instance(instance: MarkerInstance) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = instance.position_m
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        instance.quaternion_xyzw
    )
    return pose


def main() -> None:
    rclpy.init()
    node = Node("spawn_acoustic_marker")

    instance_path = Path(_token(
        node.declare_parameter("instance_path", "").value, "instance_path"))
    descriptor_path = Path(_token(
        node.declare_parameter("descriptor_path", "").value, "descriptor_path"))
    model_name = _token(node.declare_parameter("model_name", "").value, "model_name")
    allow_provisional = strict_bool(
        node.declare_parameter("allow_provisional_descriptor", False).value,
        name="allow_provisional_descriptor",
    )
    service_name = str(node.declare_parameter(
        "spawn_service", "/gazebo/spawn_sdf_model").value)
    reference_frame = _token(
        node.declare_parameter("gazebo_reference_frame", "world").value,
        "gazebo_reference_frame",
    )
    expected_instance_frame = _token(
        node.declare_parameter("expected_instance_frame", "map").value,
        "expected_instance_frame",
    )
    timeout_sec = float(node.declare_parameter("service_timeout_sec", 30.0).value)

    try:
        instance = load_instance(instance_path)
        descriptor = load_descriptor(
            descriptor_path, allow_provisional=allow_provisional
        )
        if instance.marker_id != descriptor.marker_id:
            raise ValueError("instance marker_id does not match descriptor")
        if instance.marker_revision != descriptor.revision:
            raise ValueError("instance marker_revision does not match descriptor")
        if instance.world_frame != expected_instance_frame:
            raise ValueError(
                "instance world_frame does not match the configured scenario frame"
            )
        sdf = render_sdf(descriptor, model_name=model_name)
        pose = _pose_from_instance(instance)
    except (OSError, yaml.YAMLError, ValueError) as exc:
        rospy.logfatal("acoustic marker configuration rejected: %s", exc)
        raise SystemExit(2)

    if descriptor.provisional:
        node.get_logger().warn(
            f"spawning provisional simulation-only marker {descriptor.marker_id} "
            f"revision {descriptor.revision}"
        )
    rospy.loginfo(
        "mapping marker instance frame '%s' to Gazebo reference frame '%s'",
        instance.world_frame,
        reference_frame,
    )

    # NOTE: this whole service-call block targets Gazebo Classic's /gazebo/spawn_sdf_model (gazebo_msgs/SpawnModel)
    # needs to be changed when converted to Gazebo Harmonic
    client = node.create_client(SpawnModel, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().fatal(f"marker spawn service unavailable: {service_name}")
        raise SystemExit(3)
    request = SpawnModel.Request()
    request.model_name = model_name
    request.model_xml = sdf
    request.robot_namespace = "/"
    request.initial_pose = pose
    request.reference_frame = reference_frame
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None:
        node.get_logger().fatal("marker spawn service call failed")
        raise SystemExit(3)
    if not response.success:
        node.get_logger().fatal(
            f"Gazebo rejected marker '{model_name}': {response.status_message}"
        )
        raise SystemExit(4)
    node.get_logger().info(
        f"spawned marker {model_name} ({descriptor.revision}) from {descriptor_path}"
    )
    # The launch marks this node required so a configuration or spawn failure
    # stops the marker scenario. Remain alive after the one-shot service call;
    # a clean return here would otherwise make roslaunch stop a successful run.
    rclpy.spin(node)


if __name__ == "__main__":
    main()