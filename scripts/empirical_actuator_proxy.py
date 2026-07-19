#!/usr/bin/env python3
"""Pure validation and interpolation for the simulator actuator proxy."""

import hashlib
import json
import math
from bisect import bisect_left


SCHEMA = "heron_empirical_electrical_proxy/v1"
PROXY_KIND = "current_shape_not_thrust_calibration"


def payload_sha256(payload):
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )
    return hashlib.sha256(encoded).hexdigest()


def _finite_vector(values, label):
    result = [float(value) for value in values]
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError("{} must contain finite values".format(label))
    return result


def _validate_curve(side, direction, curve):
    command = _finite_vector(curve.get("command", []), "command")
    if len(command) < 2:
        raise ValueError("{} {} curve needs at least two anchors".format(side, direction))
    if command[0] != 0.0 or command[-1] > 1.0:
        raise ValueError("{} {} command domain is invalid".format(side, direction))
    if any(a >= b for a, b in zip(command, command[1:])):
        raise ValueError("{} {} commands must increase".format(side, direction))

    for field in ("rising_relative_effort", "falling_relative_effort"):
        effort = _finite_vector(curve.get(field, []), field)
        if len(command) != len(effort):
            raise ValueError("{} {} {} is misaligned".format(side, direction, field))
        if effort[0] != 0.0 or any(value < 0.0 or value > 1.0 for value in effort):
            raise ValueError(
                "{} {} {} must remain within relative effort [0, 1]".format(
                    side, direction, field
                )
            )
        if any(a > b for a, b in zip(effort, effort[1:])):
            raise ValueError("{} {} {} must be nondecreasing".format(side, direction, field))
    for field in (
        "rising_relative_current_mad",
        "falling_relative_current_mad",
    ):
        uncertainty = _finite_vector(curve.get(field, []), field)
        if len(command) != len(uncertainty) or any(value < 0.0 for value in uncertainty):
            raise ValueError("{} {} {} is invalid".format(side, direction, field))


def validate_proxy(payload):
    """Reject ambiguous, malformed, or falsely labelled proxy artifacts."""
    if payload.get("schema") != SCHEMA:
        raise ValueError("unsupported schema")
    if payload.get("proxy_kind") != PROXY_KIND:
        raise ValueError("model must identify itself as a current-derived proxy")
    if payload.get("calibration_eligible") is not False:
        raise ValueError("simulator proxy must be ineligible as calibration evidence")
    source = payload.get("source_model")
    if not isinstance(source, dict) or int(source.get("schema_version", 0)) != 2:
        raise ValueError("source actuator model provenance is missing")
    sides = payload.get("sides")
    if not isinstance(sides, dict) or set(sides) != {"left", "right"}:
        raise ValueError("model must contain exactly left and right sides")
    for side in ("left", "right"):
        directions = sides[side].get("directions")
        if not isinstance(directions, dict) or set(directions) != {
            "forward",
            "reverse",
        }:
            raise ValueError("{} side must contain both directions".format(side))
        for direction in ("forward", "reverse"):
            _validate_curve(side, direction, directions[direction])
    return payload


def interpolate(value, xs, ys):
    if value <= xs[0]:
        return ys[0]
    if value >= xs[-1]:
        return ys[-1]
    index = bisect_left(xs, value)
    lower, upper = index - 1, index
    fraction = (value - xs[lower]) / (xs[upper] - xs[lower])
    return ys[lower] + fraction * (ys[upper] - ys[lower])


def curve_effort(curve, magnitude, sweep):
    field = "{}_relative_effort".format(sweep)
    return interpolate(float(magnitude), curve["command"], curve[field])


def select_hysteresis_sweep(
    previous_sign, previous_magnitude, previous_sweep, sign, magnitude
):
    """Retain branch state at plateaus and reset it across direction changes."""
    if sign != previous_sign:
        return "rising"
    if magnitude > previous_magnitude + 1e-12:
        return "rising"
    if magnitude < previous_magnitude - 1e-12:
        return "falling"
    return previous_sweep if previous_sweep in {"rising", "falling"} else "rising"
