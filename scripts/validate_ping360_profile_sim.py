#!/usr/bin/env python3
"""Validate the deterministic Ping360 profile simulator model."""

from __future__ import annotations

import math

from ping360_profile_model import MechanicalSweep, profile_from_points


def main():
    checks = []
    sweep = MechanicalSweep(10, 14, 2)
    assert [sweep.advance() for _ in range(6)] == [10, 12, 14, 12, 10, 12]
    checks.append("mechanical sector sweep")
    interval = 0.1
    points = [
        (5.0, 0.0, 0.0, 200.0),
        (0.0, 5.0, 0.0, 100.0),
        (float("nan"), 0.0, 0.0, 255.0),
    ]
    forward = profile_from_points(
        points, angle_grad=0, number_of_samples=100, sample_interval_m=interval
    )
    assert forward[50] == 200 and sum(1 for value in forward if value) == 1
    checks.append("angle-selective range binning")
    port = profile_from_points(
        points, angle_grad=100, number_of_samples=100, sample_interval_m=interval
    )
    assert port[50] == 100
    checks.append("native gradian convention")
    fallback = profile_from_points(
        [(2.0, 0.0, 0.0, 0.0)],
        angle_grad=0,
        number_of_samples=100,
        sample_interval_m=interval,
    )
    assert fallback[20] > 0
    checks.append("deterministic intensity fallback")
    try:
        MechanicalSweep(20, 10, 1)
    except ValueError:
        checks.append("invalid sector rejected")
    else:
        raise AssertionError("invalid sector accepted")
    print("Ping360 simulator validation passed (%d checks)" % len(checks))


if __name__ == "__main__":
    main()
