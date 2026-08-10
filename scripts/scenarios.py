#!/usr/bin/env python3
"""Resolve simulator scenario values for integration launch files."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCENARIO_INDEX_PATH = PACKAGE_DIR / "config" / "scenarios.yaml"


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


def _launch_values(
    root: Path, scenario: Mapping[str, Any], scenario_file: Path
) -> Dict[str, str]:
    spawn = dict(scenario.get("spawn_pose", {}) or {})
    offset = dict(scenario.get("world_offset", {}) or {})
    values = {
        "scenario_config_file": str(scenario_file),
        "sim_world_file": _resolve_path(root, scenario.get("world_file", "")),
        "map_entities_file": _resolve_path(root, scenario.get("entity_file", "")),
        "sim_world_offset_x": str(float(offset.get("x", 0.0) or 0.0)),
        "sim_world_offset_y": str(float(offset.get("y", 0.0) or 0.0)),
        "x": str(float(spawn.get("x", 0.0) or 0.0)),
        "y": str(float(spawn.get("y", 0.0) or 0.0)),
        "yaw": str(float(spawn.get("yaw_rad", 0.0) or 0.0)),
    }
    return values


def resolved_launch_value(*, repo_root: str, scenario: str, key: str) -> str:
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenario_file = _scenario_file(index, scenario)
    values = _launch_values(
        _workspace_root(repo_root),
        _load_yaml(scenario_file),
        scenario_file,
    )
    token = str(key or "").strip()
    if token not in values:
        raise KeyError(f"scenario has no launch value for '{token}'")
    return values[token]
