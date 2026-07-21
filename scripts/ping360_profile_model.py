#!/usr/bin/env python3
"""Pure deterministic model for canonical Ping360 simulation profiles."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple


Point = Tuple[float, float, float, float]


class MechanicalSweep:
    def __init__(self, start_angle_grad=0, stop_angle_grad=399, num_steps=1):
        if not 0 <= start_angle_grad <= stop_angle_grad <= 399:
            raise ValueError("simulation sector must satisfy 0 <= start <= stop <= 399")
        if not 1 <= num_steps <= 10:
            raise ValueError("num_steps must be in [1, 10]")
        self.start = int(start_angle_grad)
        self.stop = int(stop_angle_grad)
        self.step = int(num_steps)
        self.current = self.start
        self.direction = 1

    def advance(self) -> int:
        emitted = self.current
        candidate = self.current + self.direction * self.step
        if candidate > self.stop:
            self.direction = -1
            candidate = max(self.start, self.stop - self.step)
        elif candidate < self.start:
            self.direction = 1
            candidate = min(self.stop, self.start + self.step)
        self.current = candidate
        return emitted


def point_angle_grad(x: float, y: float) -> float:
    return (math.atan2(y, x) % (2.0 * math.pi)) * 400.0 / (2.0 * math.pi)


def circular_grad_distance(first: float, second: float) -> float:
    return abs((first - second + 200.0) % 400.0 - 200.0)


def profile_from_points(
    points: Iterable[Point],
    *,
    angle_grad: int,
    number_of_samples: int,
    sample_interval_m: float,
    beam_width_grad: float = 1.0,
) -> bytes:
    if not 0 <= angle_grad <= 399:
        raise ValueError("angle_grad must be in [0, 399]")
    if number_of_samples <= 0 or sample_interval_m <= 0.0:
        raise ValueError("positive sample geometry is required")
    intensities = bytearray(number_of_samples)
    for x, y, z, intensity in points:
        if not all(math.isfinite(value) for value in (x, y, z, intensity)):
            continue
        if (
            circular_grad_distance(point_angle_grad(x, y), angle_grad)
            > beam_width_grad / 2.0
        ):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        sample = int(distance / sample_interval_m)
        if not 0 <= sample < number_of_samples:
            continue
        strength = int(round(intensity))
        if strength <= 0:
            strength = max(1, min(255, int(round(255.0 / (1.0 + distance)))))
        intensities[sample] = max(intensities[sample], min(255, strength))
    return bytes(intensities)
