#!/usr/bin/env python3
"""Verify profile-owned defaults and explicit hydrodynamic launch overrides."""

from __future__ import annotations

import json
import math
import subprocess
import xml.etree.ElementTree as ET

import yaml


COMMON = [
    "roslaunch",
    "--dump-params",
    "grande",
    "bringup.launch",
    "mode:=sim",
    "scenario:=harbor",
    "gui:=false",
    "headless:=true",
    "use_rviz:=false",
    "use_web_viz:=false",
    "record_bags:=false",
]


def resolved(extra=()):
    output = subprocess.check_output(COMMON + list(extra), text=True)
    description = yaml.safe_load(output)["/robot_description"]
    root = ET.fromstring(description)
    # The generated model may namespace the base link. It is emitted first by
    # the Heron xacro, so the first inertial mass is the rigid hull mass.
    mass = float(root.find(".//mass").attrib["value"])
    density = float(root.find(".//fluid_density").text)
    linear = [float(value) for value in root.find(".//linear_damping").text.split()]
    quadratic = [
        float(value) for value in root.find(".//quadratic_damping").text.split()
    ]
    return {
        "mass_kg": mass,
        "fluid_density_kg_m3": density,
        "linear_damping_x": linear[0],
        "quadratic_damping_x": quadratic[0],
    }


def require_close(actual, expected, name):
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8):
        raise ValueError(f"{name}: expected {expected}, resolved {actual}")


def main():
    default = resolved()
    override = resolved(
        [
            "sim_hydrodynamics_override_enabled:=true",
            "sim_base_mass_kg:=19.50445",
            "sim_fluid_density_kg_m3:=1025.0",
            "sim_linear_damping_x:=-0.539515",
            "sim_quadratic_damping_x:=-4.314220",
        ]
    )
    expected_default = {
        "mass_kg": 42.0,
        "fluid_density_kg_m3": 1025.0,
        "linear_damping_x": -30.0,
        "quadratic_damping_x": -15.0,
    }
    expected_override = {
        "mass_kg": 19.50445,
        "fluid_density_kg_m3": 1025.0,
        "linear_damping_x": -0.539515,
        "quadratic_damping_x": -4.314220,
    }
    for name, expected in expected_default.items():
        require_close(default[name], expected, f"default {name}")
    for name, expected in expected_override.items():
        require_close(override[name], expected, f"override {name}")
    print(
        json.dumps(
            {"profile_default": default, "explicit_override": override}, indent=2
        )
    )


if __name__ == "__main__":
    main()
