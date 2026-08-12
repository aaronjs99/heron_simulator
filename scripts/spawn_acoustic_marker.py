#!/usr/bin/env python3
"""Spawn one descriptor-driven acoustic marker in Gazebo."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Mapping, Tuple

import rospy
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from acoustic_marker_model import (  # noqa: E402
    MarkerDescriptorError,
    load_descriptor,
    render_sdf,
)


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


def _boolean_param(name: str, default: bool) -> bool:
    value = rospy.get_param(name, default)
    if not isinstance(value, bool):
        raise MarkerInstanceError(f"{name} must be a boolean")
    return value


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
    rospy.init_node("spawn_acoustic_marker")
    instance_path = Path(_token(rospy.get_param("~instance_path", ""), "instance_path"))
    descriptor_path = Path(
        _token(rospy.get_param("~descriptor_path", ""), "descriptor_path")
    )
    model_name = _token(rospy.get_param("~model_name", ""), "model_name")
    allow_provisional = _boolean_param("~allow_provisional_descriptor", False)
    service_name = str(rospy.get_param("~spawn_service", "/gazebo/spawn_sdf_model"))
    reference_frame = _token(
        rospy.get_param("~gazebo_reference_frame", "world"),
        "gazebo_reference_frame",
    )
    expected_instance_frame = _token(
        rospy.get_param("~expected_instance_frame", "map"),
        "expected_instance_frame",
    )
    timeout_sec = float(rospy.get_param("~service_timeout_sec", 30.0))
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
        rospy.logfatal("acoustic marker configuration rejected: %s", exc)
        raise SystemExit(2)

    if descriptor.provisional:
        rospy.logwarn(
            "spawning provisional simulation-only marker %s revision %s",
            descriptor.marker_id,
            descriptor.revision,
        )
    rospy.loginfo(
        "mapping marker instance frame '%s' to Gazebo reference frame '%s'",
        instance["world_frame"],
        reference_frame,
    )
    try:
        rospy.wait_for_service(service_name, timeout=timeout_sec)
        spawn = rospy.ServiceProxy(service_name, SpawnModel)
        response = spawn(model_name, sdf, "/", pose, reference_frame)
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logfatal("marker spawn service failed: %s", exc)
        raise SystemExit(3)
    if not response.success:
        rospy.logfatal(
            "Gazebo rejected marker '%s': %s", model_name, response.status_message
        )
        raise SystemExit(4)
    rospy.loginfo(
        "spawned marker %s (%s) from %s",
        model_name,
        descriptor.revision,
        descriptor_path,
    )
    # The launch marks this node required so a configuration or spawn failure
    # stops the marker scenario. Remain alive after the one-shot service call;
    # a clean return here would otherwise make roslaunch stop a successful run.
    rospy.spin()


if __name__ == "__main__":
    main()
