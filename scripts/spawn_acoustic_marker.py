#!/usr/bin/env python3
"""Spawn one descriptor-driven acoustic marker in Gazebo."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Tuple

import rclpy
from rclpy.node import Node
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

from models.acoustic_marker_model import (
    MarkerDescriptorError,
    load_descriptor,
    render_sdf,
)
from models.parameters import strict_bool


class MarkerInstanceError(ValueError):
    """Raised when a simulator marker instance is incomplete."""


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MarkerInstanceError(f"{label} must be a mapping")
    return value


def _token(value: Any, label: str) -> str:
    token = str(value or "").strip()
    if not token:
        raise MarkerInstanceError(f"{label} must be non-empty")
    return token


def _number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MarkerInstanceError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise MarkerInstanceError(f"{label} must be finite")
    return result


def _xyz(value: Any, label: str) -> Tuple[float, float, float]:
    mapping = _mapping(value, label)
    missing = {axis for axis in ("x", "y", "z") if axis not in mapping}
    if missing:
        raise MarkerInstanceError(f"{label} is missing {', '.join(sorted(missing))}")
    return tuple(_number(mapping[axis], f"{label}.{axis}") for axis in ("x", "y", "z"))


def _quaternion(value: Any, label: str) -> Tuple[float, float, float, float]:
    mapping = _mapping(value, label)
    missing = {axis for axis in ("x", "y", "z", "w") if axis not in mapping}
    if missing:
        raise MarkerInstanceError(f"{label} is missing {', '.join(sorted(missing))}")
    quaternion = tuple(
        _number(mapping[axis], f"{label}.{axis}") for axis in ("x", "y", "z", "w")
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.0) > 1.0e-6:
        raise MarkerInstanceError(f"{label} must be a unit quaternion")
    return quaternion


def load_instance(path: Path) -> Mapping[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    root = _mapping(document, "instance document")
    schema_version = root.get("schema_version")
    if isinstance(schema_version, bool) or schema_version != 1:
        raise MarkerInstanceError("marker instance schema_version must equal 1")
    instance = _mapping(root.get("instance"), "instance")
    _token(instance.get("id"), "instance.id")
    _token(instance.get("marker_id"), "instance.marker_id")
    _token(instance.get("marker_revision"), "instance.marker_revision")
    _token(instance.get("world_frame"), "instance.world_frame")
    pose = _mapping(instance.get("pose"), "instance.pose")
    _xyz(pose.get("position_m"), "instance.pose.position_m")
    _quaternion(pose.get("quaternion_xyzw"), "instance.pose.quaternion_xyzw")
    if not isinstance(instance.get("surveyed"), bool):
        raise MarkerInstanceError("instance.surveyed must be boolean")
    if not isinstance(instance.get("runtime_approved"), bool):
        raise MarkerInstanceError("instance.runtime_approved must be boolean")
    return instance


def _pose_from_instance(instance: Mapping[str, Any]) -> Pose:
    value = _mapping(instance.get("pose"), "instance.pose")
    position = _xyz(value.get("position_m"), "instance.pose.position_m")
    quaternion = _quaternion(
        value.get("quaternion_xyzw"), "instance.pose.quaternion_xyzw"
    )
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        quaternion
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
        if instance["marker_id"] != descriptor.marker_id:
            raise MarkerInstanceError("instance marker_id does not match descriptor")
        if instance["marker_revision"] != descriptor.revision:
            raise MarkerInstanceError(
                "instance marker_revision does not match descriptor"
            )
        if instance["world_frame"] != expected_instance_frame:
            raise MarkerInstanceError(
                "instance world_frame does not match the configured scenario frame"
            )
        sdf = render_sdf(descriptor, model_name=model_name)
        pose = _pose_from_instance(instance)
    except (OSError, yaml.YAMLError, MarkerDescriptorError, MarkerInstanceError) as exc:
        node.get_logger().fatal(f"acoustic marker configuration rejected: {exc}")
        raise SystemExit(2)

    if descriptor.provisional:
        node.get_logger().warn(
            f"spawning provisional simulation-only marker {descriptor.marker_id} "
            f"revision {descriptor.revision}"
        )
    node.get_logger().info(
        f"mapping marker instance frame '{instance['world_frame']}' to Gazebo "
        f"reference frame '{reference_frame}'"
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