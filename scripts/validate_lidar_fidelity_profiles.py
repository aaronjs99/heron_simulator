#!/usr/bin/env python3
"""Validate full-fidelity defaults and opt-in lidar performance controls."""

from __future__ import annotations

import json
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def require(condition: bool, label: str, checks: list[str]) -> None:
    if not condition:
        raise ValueError(label)
    checks.append(label)


def main() -> int:
    checks: list[str] = []
    spawn = (PACKAGE_ROOT / "launch" / "spawn_heron.launch").read_text(encoding="utf-8")
    world = (PACKAGE_ROOT / "launch" / "heron_world.launch").read_text(encoding="utf-8")
    sensors = (PACKAGE_ROOT / "urdf" / "sensors.urdf.xacro").read_text(encoding="utf-8")

    defaults = {
        "lidar_update_rate_hz": "10",
        "lidar_horizontal_samples": "1800",
        "lidar_vertical_samples": "16",
    }
    env_names = {
        "lidar_update_rate_hz": "HERON_SIM_LIDAR_UPDATE_RATE_HZ",
        "lidar_horizontal_samples": "HERON_SIM_LIDAR_HORIZONTAL_SAMPLES",
        "lidar_vertical_samples": "HERON_SIM_LIDAR_VERTICAL_SAMPLES",
    }
    for arg, default in defaults.items():
        require(
            f'<arg name="{arg}" default="{default}"' in spawn,
            f"spawn preserves {arg} full-fidelity default",
            checks,
        )
        require(
            f'<arg name="{arg}" default="{default}"' in world,
            f"world preserves {arg} full-fidelity default",
            checks,
        )
        require(
            f"{env_names[arg]}=$(arg {arg})" in spawn,
            f"spawn exports {arg}",
            checks,
        )
        require(
            f'<arg name="{arg}" value="$(arg {arg})"' in world,
            f"world forwards {arg}",
            checks,
        )
        require(
            f"$(optenv {env_names[arg]} {default})" in sensors,
            f"sensor model consumes {arg} with fidelity default",
            checks,
        )

    require(
        sensors.count("HERON_SIM_LIDAR_UPDATE_RATE_HZ") == 2,
        "both lidars share configured update rate",
        checks,
    )
    require(
        sensors.count("HERON_SIM_LIDAR_HORIZONTAL_SAMPLES") == 2,
        "both lidars share configured horizontal sampling",
        checks,
    )
    require(
        sensors.count("HERON_SIM_LIDAR_VERTICAL_SAMPLES") == 2,
        "both lidars share configured vertical sampling",
        checks,
    )
    print(json.dumps({"checks": len(checks), "defaults": defaults}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
