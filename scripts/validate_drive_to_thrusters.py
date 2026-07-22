#!/usr/bin/env python3
"""Focused simulator actuator direction and time-epoch regression checks."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys

import rospy
import rospkg
import yaml

ROOT = Path(rospkg.RosPack().get_path("heron_simulator"))
sys.path.insert(0, str(ROOT / "scripts"))
from empirical_actuator_proxy import validate_proxy  # noqa: E402


def load_bridge_class():
    path = ROOT / "scripts" / "drive_to_thrusters.py"
    spec = importlib.util.spec_from_file_location("drive_to_thrusters", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DriveToThrusters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--actuator-proxy", type=Path, required=True)
    parser.add_argument(
        "--dynamics-config",
        type=Path,
        default=ROOT / "config/thruster_dynamics_session4_proxy.yaml",
    )
    args = parser.parse_args()

    payload = validate_proxy(
        json.loads(args.actuator_proxy.read_text(encoding="utf-8"))
    )
    dynamics = yaml.safe_load(args.dynamics_config.read_text(encoding="utf-8"))
    bridge_class = load_bridge_class()
    bridge = bridge_class.__new__(bridge_class)
    bridge.empirical_model = payload
    bridge.reference_current_a = float(payload["normalization"]["reference_current_a"])
    bridge.max_fwd_thrust = float(dynamics["max_fwd_thrust"])
    bridge.max_bck_thrust = float(dynamics["max_bck_thrust"])
    bridge.forward_current_scale = float(dynamics["forward_current_scale"])
    bridge.reverse_current_scale = float(dynamics["reverse_current_scale"])
    bridge.synthetic_current_a = {"left": 0.0, "right": 0.0}
    bridge.previous_curve_magnitude = {"left": 0.0, "right": 0.0}
    bridge.previous_curve_sign = {"left": 0, "right": 0}
    bridge.previous_curve_sweep = {"left": "rising", "right": "rising"}

    reverse_force = bridge.drive_to_thrust(-0.35, "left")
    reverse_current = bridge.synthetic_current_a["left"]
    forward_force = bridge.drive_to_thrust(0.30, "left")
    forward_current = bridge.synthetic_current_a["left"]
    if not (reverse_force < 0.0 and reverse_current > 0.0):
        raise SystemExit(
            "reverse sensitivity must produce negative force and positive current"
        )
    if not (forward_force > 0.0 and forward_current > 0.0):
        raise SystemExit("Session 4 forward replay must produce positive force/current")

    bridge.target_left = 0.4
    bridge.target_right = -0.2
    bridge.actual_left = 0.3
    bridge.actual_right = -0.1
    bridge.synthetic_current_a = {"left": 2.0, "right": 1.0}
    bridge.previous_curve_magnitude = {"left": 0.4, "right": 0.2}
    bridge.previous_curve_sign = {"left": 1, "right": -1}
    bridge.previous_curve_sweep = {"left": "falling", "right": "falling"}
    now = rospy.Time.from_sec(5.0)
    bridge.reset_actuator_epoch(now)
    if any(
        (
            bridge.target_left,
            bridge.target_right,
            bridge.actual_left,
            bridge.actual_right,
            bridge.synthetic_current_a["left"],
            bridge.synthetic_current_a["right"],
        )
    ):
        raise SystemExit("time-epoch reset retained actuator state")
    if bridge.last_cmd_time != now or bridge.last_update_time != now:
        raise SystemExit("time-epoch reset did not re-anchor timestamps")

    print(
        json.dumps(
            {
                "reverse_negative_force_positive_current": True,
                "forward_positive_force_current": True,
                "time_rollback_clears_state": True,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
