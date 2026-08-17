#!/usr/bin/env python3
"""Resolve simulator scenario values for integration launch files."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

import rospkg
import yaml

from heron_simulator_runtime.parameters import strict_bool

ROS_PACK = rospkg.RosPack()


def _package_dir() -> Path:
    try:
        return Path(ROS_PACK.get_path("heron_simulator"))
    except rospkg.ResourceNotFound:
        return Path(__file__).resolve().parents[1]


PACKAGE_DIR = _package_dir()
SCENARIO_INDEX_PATH = PACKAGE_DIR / "config" / "scenarios.yaml"


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def _resolve_path(value: str) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    path = Path(token).expanduser()
    if path.is_absolute():
        return str(path)
    parts = path.parts
    if len(parts) > 1:
        try:
            package_root = Path(ROS_PACK.get_path(parts[0]))
        except rospkg.ResourceNotFound:
            package_root = PACKAGE_DIR.parent / parts[0]
        return str(package_root.joinpath(*parts[1:]))
    return str(PACKAGE_DIR / path)


def _scenario_file(index: Mapping[str, Any], name: str) -> Path:
    scenarios = dict(index.get("scenarios", {}) or {})
    selected = str(name or index.get("default_scenario", "harbor") or "harbor")
    value = scenarios.get(selected)
    if not value:
        available = ", ".join(sorted(scenarios)) or "<none>"
        raise KeyError(f"unknown simulator scenario '{selected}' ({available})")
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else PACKAGE_DIR / path


def _launch_values(scenario: Mapping[str, Any], scenario_file: Path) -> Dict[str, str]:
    spawn = dict(scenario.get("spawn_pose", {}) or {})
    offset = dict(scenario.get("world_offset", {}) or {})
    values = {
        "scenario_config_file": str(scenario_file),
        "sim_world_file": _resolve_path(scenario.get("world_file", "")),
        "map_entities_file": _resolve_path(scenario.get("entity_file", "")),
        "sim_world_offset_x": str(float(offset.get("x", 0.0) or 0.0)),
        "sim_world_offset_y": str(float(offset.get("y", 0.0) or 0.0)),
        "x": str(float(spawn.get("x", 0.0) or 0.0)),
        "y": str(float(spawn.get("y", 0.0) or 0.0)),
        "yaw": str(float(spawn.get("yaw_rad", 0.0) or 0.0)),
        "spawn_acoustic_marker": str(
            strict_bool(
                scenario.get("spawn_acoustic_marker", False),
                name="scenario.spawn_acoustic_marker",
            )
        ).lower(),
        "acoustic_marker_instance_file": _resolve_path(
            scenario.get("marker_instance_file", "")
        ),
        "acoustic_marker_descriptor_file": _resolve_path(
            scenario.get("marker_descriptor_file", "")
        ),
        "acoustic_marker_model_name": str(scenario.get("marker_model_name", "") or ""),
    }
    return values


def resolved_launch_value(*, scenario: str, key: str) -> str:
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenario_file = _scenario_file(index, scenario)
    values = _launch_values(_load_yaml(scenario_file), scenario_file)
    token = str(key or "").strip()
    if token not in values:
        raise KeyError(f"scenario has no launch value for '{token}'")
    return values[token]
