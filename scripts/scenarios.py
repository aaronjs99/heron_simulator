#!/usr/bin/env python3
"""Load validated simulator scenario configuration bundles."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

import rospkg
import yaml

from models.parameters import strict_bool

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


def scenario_names() -> Tuple[str, ...]:
    """Return every configured scenario name in deterministic order."""
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenarios = dict(index.get("scenarios", {}) or {})
    if not scenarios:
        raise ValueError(f"scenario index is empty: {SCENARIO_INDEX_PATH}")
    return tuple(sorted(str(name) for name in scenarios))


def scenario_launch_values(scenario: str) -> Dict[str, str]:
    """Resolve one coherent, validated launch bundle for ``scenario``."""
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenario_file = _scenario_file(index, scenario)
    if not scenario_file.is_file():
        raise FileNotFoundError(f"scenario configuration is missing: {scenario_file}")
    values = _launch_values(_load_yaml(scenario_file), scenario_file)
    required_files = {
        "sim_world_file": values["sim_world_file"],
        "map_entities_file": values["map_entities_file"],
    }
    if values["spawn_acoustic_marker"] == "true":
        required_files.update(
            {
                "acoustic_marker_instance_file": values[
                    "acoustic_marker_instance_file"
                ],
                "acoustic_marker_descriptor_file": values[
                    "acoustic_marker_descriptor_file"
                ],
            }
        )
        if not values["acoustic_marker_model_name"].strip():
            raise ValueError(
                f"scenario {scenario!r} enables a marker without a model name"
            )
    for field, raw_path in required_files.items():
        if not raw_path or not Path(raw_path).is_file():
            raise FileNotFoundError(
                f"scenario {scenario!r} has missing {field}: {raw_path or '<empty>'}"
            )
    return values
