#!/usr/bin/env python3
"""Focused checks for scenario launch-value resolution."""

import os
import sys

import yaml


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM_ROOT = os.path.join(REPO_ROOT, "heron_simulator")
SCRIPTS_ROOT = os.path.join(SIM_ROOT, "scripts")
if SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, SCRIPTS_ROOT)

from scenarios import resolved_launch_value  # noqa: E402


def test_harbor_scenario_config_file_resolves_to_yaml():
    path = resolved_launch_value(
        repo_root=REPO_ROOT,
        scenario="harbor",
        key="scenario_config_file",
    )

    assert path.endswith("heron_simulator/config/scenarios/harbor.yaml")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    exploration = data["exploration"]
    assert exploration["required_extent_m"] == 3.0
    assert exploration["precommit_clearance_m"] == 1.0
    assert exploration["progressive_distance_cap_m"] == 4.0


def test_harbor_progressive_cap_still_resolves_as_launch_arg():
    value = resolved_launch_value(
        repo_root=REPO_ROOT,
        scenario="harbor",
        key="exploration_goal_progressive_distance_cap_m",
    )

    assert value == "4.0"
