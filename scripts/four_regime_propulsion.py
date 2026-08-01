#!/usr/bin/env python3
"""Pure four-regime propulsion plant used by runtime and validation."""

import math


def clamp(value, lower, upper):
    return lower if value < lower else upper if value > upper else value


def normalized_effort(drive, deadband, exponent):
    magnitude = abs(clamp(float(drive), -1.0, 1.0))
    deadband = clamp(float(deadband), 0.0, 0.95)
    exponent = max(0.1, float(exponent))
    if magnitude <= deadband:
        return 0.0
    return ((magnitude - deadband) / (1.0 - deadband)) ** exponent


def propulsion_output(drive, regime, voltage_v):
    """Return deterministic force and electrical proxy values for one regime."""
    drive = clamp(float(drive), -1.0, 1.0)
    if abs(drive) <= 1e-12:
        return {
            "force_n": 0.0,
            "current_a": 0.0,
            "rpm": 0.0,
            "pwm_us": 1500.0,
            "effort": 0.0,
        }
    effort = normalized_effort(
        drive, regime.get("deadband", 0.0), regime.get("force_exponent", 1.0)
    )
    sign = 1.0 if drive > 0.0 else -1.0
    nominal_voltage = max(1e-9, float(regime.get("nominal_voltage_v", 16.0)))
    voltage_scale = max(0.25, float(voltage_v) / nominal_voltage) ** float(
        regime.get("voltage_exponent", 2.0)
    )
    return {
        "force_n": sign
        * effort
        * max(0.0, float(regime["max_force_n"]))
        * voltage_scale,
        "current_a": effort * max(0.0, float(regime["max_current_a"])),
        "rpm": sign * math.sqrt(effort) * max(0.0, float(regime.get("max_rpm", 0.0))),
        "pwm_us": 1500.0 + sign * 500.0 * abs(drive),
        "effort": effort,
    }
