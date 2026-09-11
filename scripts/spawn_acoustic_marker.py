#!/usr/bin/env python3
"""Spawn one descriptor-driven acoustic marker in Gazebo."""

from __future__ import annotations

from pathlib import Path

import rospy
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
    rospy.init_node("spawn_acoustic_marker")
    instance_path = Path(_token(rospy.get_param("~instance_path", ""), "instance_path"))
    descriptor_path = Path(
        _token(rospy.get_param("~descriptor_path", ""), "descriptor_path")
    )
    model_name = _token(rospy.get_param("~model_name", ""), "model_name")
    allow_provisional = strict_bool(
        rospy.get_param("~allow_provisional_descriptor", False),
        name="~allow_provisional_descriptor",
    )
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
        rospy.logwarn(
            "spawning provisional simulation-only marker %s revision %s",
            descriptor.marker_id,
            descriptor.revision,
        )
    rospy.loginfo(
        "mapping marker instance frame '%s' to Gazebo reference frame '%s'",
        instance.world_frame,
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
