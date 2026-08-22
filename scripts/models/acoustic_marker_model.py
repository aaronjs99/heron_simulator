#!/usr/bin/env python3
"""Validate a canonical range-marker descriptor and render deterministic SDF.

The descriptor is owned by ``range_aid``.  This module gives Gazebo a geometry
adapter only: ``laser_retro`` values are deterministic ray-intensity
surrogates, not acoustic reflectivity or a sonar-performance model.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple
from xml.dom import minidom

import yaml
from range_aid.marker.model import (
    MarkerDescriptor,
    SphereElement as Sphere,
    StrutElement as Strut,
    descriptor_from_mapping as validate_descriptor_mapping,
)


class MarkerDescriptorError(ValueError):
    """Raised when a marker descriptor cannot define an unambiguous model."""


_SDF_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_-]*$")


def _token(value: Any, label: str) -> str:
    token = str(value or "").strip()
    if not token:
        raise MarkerDescriptorError(f"{label} must be non-empty")
    return token


def _identifier(value: Any, label: str) -> str:
    token = _token(value, label)
    if not _SDF_IDENTIFIER.fullmatch(token):
        raise MarkerDescriptorError(
            f"{label} must start with a letter or underscore and contain only "
            "letters, digits, underscores, or hyphens"
        )
    return token


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _validated_descriptor(
    document: Mapping[str, Any],
    *,
    source: str,
    allow_provisional: bool,
) -> MarkerDescriptor:
    """Apply the canonical RANGE_AID contract, then SDF-only constraints."""

    try:
        descriptor = validate_descriptor_mapping(document, source)
    except (OverflowError, TypeError, ValueError) as exc:
        raise MarkerDescriptorError(str(exc)) from exc
    if descriptor.provisional and not allow_provisional:
        raise MarkerDescriptorError(
            "provisional marker rejected; simulation must opt in explicitly"
        )
    _identifier(descriptor.marker_id, "marker.id")
    for index, sphere in enumerate(descriptor.spheres):
        _identifier(sphere.element_id, f"elements.spheres[{index}].id")
    for index, strut in enumerate(descriptor.struts):
        _identifier(strut.element_id, f"elements.struts[{index}].id")
    return descriptor


def descriptor_from_mapping(
    document: Mapping[str, Any], *, allow_provisional: bool = False
) -> MarkerDescriptor:
    """Validate canonically, then enforce simulator-only SDF constraints."""

    return _validated_descriptor(
        document,
        source="descriptor",
        allow_provisional=allow_provisional,
    )


def load_descriptor(path: Path, *, allow_provisional: bool = False) -> MarkerDescriptor:
    with Path(path).open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    return _validated_descriptor(
        document,
        source=str(path),
        allow_provisional=allow_provisional,
    )


def _format_number(value: float) -> str:
    if abs(value) < 5.0e-13:
        value = 0.0
    return f"{value:.9g}"


def _pose_text(
    position: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)
) -> str:
    return " ".join(_format_number(value) for value in (*position, *rpy))


def _append_text(document: minidom.Document, parent: Any, tag: str, value: Any) -> Any:
    element = document.createElement(tag)
    element.appendChild(document.createTextNode(str(value)))
    parent.appendChild(element)
    return element


def _append_geometry(
    document: minidom.Document,
    parent: Any,
    *,
    shape: str,
    radius_m: float,
    length_m: float = 0.0,
) -> None:
    geometry = document.createElement("geometry")
    body = document.createElement(shape)
    _append_text(document, body, "radius", _format_number(radius_m))
    if shape == "cylinder":
        _append_text(document, body, "length", _format_number(length_m))
    geometry.appendChild(body)
    parent.appendChild(geometry)


def _append_material(
    document: minidom.Document, visual: Any, color: Sequence[float]
) -> None:
    material = document.createElement("material")
    rgba = " ".join(_format_number(value) for value in color)
    _append_text(document, material, "ambient", rgba)
    _append_text(document, material, "diffuse", rgba)
    visual.appendChild(material)


def _strut_pose(
    strut: Strut, sphere_by_id: Mapping[str, Sphere]
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], float]:
    first = sphere_by_id[strut.from_sphere]
    second = sphere_by_id[strut.to_sphere]
    vector = tuple(b - a for a, b in zip(first.center_m, second.center_m))
    distance = math.sqrt(sum(value * value for value in vector))
    direction = tuple(value / distance for value in vector)
    start = tuple(
        center + first.radius_m * axis
        for center, axis in zip(first.center_m, direction)
    )
    end = tuple(
        center - second.radius_m * axis
        for center, axis in zip(second.center_m, direction)
    )
    length = _distance(start, end)
    if length <= 0.0:
        raise MarkerDescriptorError(f"strut '{strut.element_id}' has no exposed length")
    midpoint = tuple((a + b) * 0.5 for a, b in zip(start, end))
    horizontal = math.hypot(vector[0], vector[1])
    pitch = math.atan2(horizontal, vector[2])
    yaw = math.atan2(vector[1], vector[0]) if horizontal > 0.0 else 0.0
    return midpoint, (0.0, pitch, yaw), length


def render_sdf(descriptor: MarkerDescriptor, *, model_name: str) -> str:
    """Render one static Gazebo model in stable ID order."""

    model_token = _identifier(model_name, "model_name")
    document = minidom.Document()
    sdf = document.createElement("sdf")
    sdf.setAttribute("version", "1.6")
    document.appendChild(sdf)
    model = document.createElement("model")
    model.setAttribute("name", model_token)
    sdf.appendChild(model)
    _append_text(document, model, "static", "true")
    link = document.createElement("link")
    link.setAttribute("name", "marker")
    model.appendChild(link)

    palette = (
        (0.95, 0.24, 0.20, 1.0),
        (0.98, 0.62, 0.12, 1.0),
        (0.96, 0.87, 0.22, 1.0),
        (0.22, 0.72, 0.64, 1.0),
        (0.20, 0.48, 0.82, 1.0),
        (0.58, 0.36, 0.78, 1.0),
    )
    spheres = sorted(descriptor.spheres, key=lambda item: item.element_id)
    struts = sorted(descriptor.struts, key=lambda item: item.element_id)
    for index, sphere in enumerate(spheres):
        collision = document.createElement("collision")
        collision.setAttribute("name", f"sphere_{sphere.element_id}_collision")
        _append_text(document, collision, "pose", _pose_text(sphere.center_m))
        _append_geometry(
            document,
            collision,
            shape="sphere",
            radius_m=sphere.radius_m,
        )
        _append_text(
            document,
            collision,
            "laser_retro",
            _format_number(sphere.sim_ray_intensity),
        )
        link.appendChild(collision)

        visual = document.createElement("visual")
        visual.setAttribute("name", f"sphere_{sphere.element_id}_visual")
        _append_text(document, visual, "pose", _pose_text(sphere.center_m))
        _append_geometry(
            document,
            visual,
            shape="sphere",
            radius_m=sphere.radius_m,
        )
        _append_material(document, visual, palette[index % len(palette)])
        link.appendChild(visual)

    sphere_by_id = {sphere.element_id: sphere for sphere in spheres}
    for strut in struts:
        position, rpy, length = _strut_pose(strut, sphere_by_id)
        collision = document.createElement("collision")
        collision.setAttribute("name", f"strut_{strut.element_id}_collision")
        _append_text(document, collision, "pose", _pose_text(position, rpy))
        _append_geometry(
            document,
            collision,
            shape="cylinder",
            radius_m=strut.radius_m,
            length_m=length,
        )
        _append_text(
            document,
            collision,
            "laser_retro",
            _format_number(strut.sim_ray_intensity),
        )
        link.appendChild(collision)

        visual = document.createElement("visual")
        visual.setAttribute("name", f"strut_{strut.element_id}_visual")
        _append_text(document, visual, "pose", _pose_text(position, rpy))
        _append_geometry(
            document,
            visual,
            shape="cylinder",
            radius_m=strut.radius_m,
            length_m=length,
        )
        _append_material(document, visual, (0.35, 0.35, 0.38, 1.0))
        link.appendChild(visual)

    raw = document.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")
    return "\n".join(line for line in raw.splitlines() if line.strip()) + "\n"


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render a canonical RANGE_AID marker descriptor as Gazebo SDF."
    )
    parser.add_argument("descriptor", type=Path)
    parser.add_argument("--model-name", default="orion_planetary_marker")
    parser.add_argument("--allow-provisional", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        descriptor = load_descriptor(
            args.descriptor, allow_provisional=args.allow_provisional
        )
        rendered = render_sdf(descriptor, model_name=args.model_name)
    except (OSError, yaml.YAMLError, MarkerDescriptorError) as exc:
        parser.error(str(exc))
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        sys.stdout.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
