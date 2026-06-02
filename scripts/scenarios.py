#!/usr/bin/env python3
"""Resolve simulator scenario values for integration launch files."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCENARIO_INDEX_PATH = PACKAGE_DIR / "config" / "scenarios.yaml"

LAUNCH_ARG_KEYS = {
    "max_generated_goals": "exploration_max_generated_goals",
    "spacing_m": "exploration_spacing",
    "radius_m": "exploration_radius",
    "min_frontier_progress_m": "exploration_min_frontier_progress_m",
    "min_total_exploration_radius_m": "exploration_min_total_exploration_radius_m",
    "goal_current_pose_tolerance_m": "exploration_goal_current_pose_tolerance_m",
    "goal_min_displacement_m": "exploration_goal_min_displacement_m",
    "goal_min_obstacle_clearance_m": "exploration_goal_min_obstacle_clearance_m",
    "goal_min_frontier_standoff_m": "exploration_goal_min_frontier_standoff_m",
    "goal_preferred_frontier_standoff_m": "exploration_goal_preferred_frontier_standoff_m",
    "goal_map_bounds_margin_m": "exploration_goal_map_bounds_margin_m",
    "goal_standoff_projection_step_m": "exploration_goal_standoff_projection_step_m",
    "goal_standoff_projection_max_m": "exploration_goal_standoff_projection_max_m",
}


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def _workspace_root(repo_root: str) -> Path:
    token = Path(str(repo_root or "")).expanduser()
    if token.is_absolute() and token.exists():
        return token
    return PACKAGE_DIR.parent


def _resolve_path(root: Path, value: str) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    path = Path(token).expanduser()
    return str(path if path.is_absolute() else root / path)


def _scenario_file(index: Mapping[str, Any], name: str) -> Path:
    scenarios = dict(index.get("scenarios", {}) or {})
    selected = str(name or index.get("default_scenario", "harbor") or "harbor")
    value = scenarios.get(selected)
    if not value:
        available = ", ".join(sorted(scenarios)) or "<none>"
        raise KeyError(f"unknown simulator scenario '{selected}' ({available})")
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else PACKAGE_DIR / path


def _scenario(index: Mapping[str, Any], name: str) -> Dict[str, Any]:
    return _load_yaml(_scenario_file(index, name))


def _launch_values(root: Path, scenario: Mapping[str, Any]) -> Dict[str, str]:
    spawn = dict(scenario.get("spawn_pose", {}) or {})
    offset = dict(scenario.get("world_offset", {}) or {})
    costmap = dict(scenario.get("costmap_config", {}) or {})
    exploration = dict(scenario.get("exploration", {}) or {})
    values = {
        "sim_world_file": _resolve_path(root, scenario.get("world_file", "")),
        "map_entities_file": _resolve_path(root, scenario.get("entity_file", "")),
        "sim_world_offset_x": str(float(offset.get("x", 0.0) or 0.0)),
        "sim_world_offset_y": str(float(offset.get("y", 0.0) or 0.0)),
        "global_costmap_config": _resolve_path(root, costmap.get("global", "")),
        "local_costmap_config": _resolve_path(root, costmap.get("local", "")),
        "global_costmap_overlay_config": _resolve_path(
            root, costmap.get("global_overlay", "")
        ),
        "local_costmap_overlay_config": _resolve_path(
            root, costmap.get("local_overlay", "")
        ),
        "teb_local_planner_overlay_config": _resolve_path(
            root, costmap.get("teb_overlay", "")
        ),
        "x": str(float(spawn.get("x", 0.0) or 0.0)),
        "y": str(float(spawn.get("y", 0.0) or 0.0)),
        "yaw": str(float(spawn.get("yaw_rad", 0.0) or 0.0)),
    }
    for source_key, launch_key in LAUNCH_ARG_KEYS.items():
        if source_key in exploration and exploration[source_key] is not None:
            values[launch_key] = str(exploration[source_key])
    return values


def resolved_launch_value(*, repo_root: str, scenario: str, key: str) -> str:
    values = _launch_values(
        _workspace_root(repo_root),
        _scenario(_load_yaml(SCENARIO_INDEX_PATH), scenario),
    )
    token = str(key or "").strip()
    if token not in values:
        raise KeyError(f"scenario has no launch value for '{token}'")
    return values[token]
