#!/usr/bin/env python3
"""Load validated simulator scenario configuration bundles."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple
import math
import struct
import xml.etree.ElementTree as ET

import rospkg
import yaml

from models.parameters import strict_bool

ROS_PACK = rospkg.RosPack()


def _package_dir() -> Path:
    try:
        return Path(ROS_PACK.get_path("heron_simulator"))
    except rospkg.ResourceNotFound:
        return Path(__file__).resolve().parents[2]


PACKAGE_DIR = _package_dir()
SCENARIO_INDEX_PATH = PACKAGE_DIR / "config" / "scenarios" / "index.yaml"


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


@lru_cache(maxsize=None)
def _package_path(name: str) -> Path:
    try:
        return Path(ROS_PACK.get_path(name))
    except rospkg.ResourceNotFound:
        source_root = next(
            (
                path
                for path in (PACKAGE_DIR, *PACKAGE_DIR.parents)
                if path.name == "src"
            ),
            PACKAGE_DIR.parent,
        )
        for manifest in source_root.rglob("package.xml"):
            try:
                if ET.parse(str(manifest)).getroot().findtext("name") == name:
                    return manifest.parent
            except ET.ParseError:
                continue
        raise FileNotFoundError("ROS package is unavailable: " + name)


def _resolve_path(value: str) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    path = Path(token).expanduser()
    if path.is_absolute():
        return str(path)
    parts = path.parts
    if len(parts) > 1:
        package_root = _package_path(parts[0])
        return str(package_root.joinpath(*parts[1:]))
    return str(PACKAGE_DIR / path)


def _scenario_file(index: Mapping[str, Any], name: str) -> Path:
    scenarios = dict(index.get("scenarios", {}) or {})
    selected = str(name or index.get("default_scenario", "harbor") or "harbor")
    external = Path(selected).expanduser()
    if external.is_file():
        return external.resolve()
    value = scenarios.get(selected)
    if not value:
        available = ", ".join(sorted(scenarios)) or "<none>"
        raise KeyError(f"unknown simulator scenario '{selected}' ({available})")
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else PACKAGE_DIR / path


def _launch_values(scenario: Mapping[str, Any], scenario_file: Path) -> Dict[str, str]:
    spawn = dict(scenario.get("spawn_pose", {}) or {})
    offset = dict(scenario.get("world_offset", {}) or {})
    values = {
        "scenario_config_file": str(scenario_file),
        "sim_world_file": _resolve_path(scenario.get("world_file", "")),
        "map_entities_file": _resolve_path(scenario.get("entity_file", "")),
        "sim_world_offset_x": str(float(offset.get("x", 0.0) or 0.0)),
        "sim_world_offset_y": str(float(offset.get("y", 0.0) or 0.0)),
        "x": str(float(spawn.get("x", 0.0) or 0.0)),
        "y": str(float(spawn.get("y", 0.0) or 0.0)),
        "yaw": str(float(spawn.get("yaw_rad", 0.0) or 0.0)),
        "spawn_acoustic_marker": str(
            strict_bool(
                scenario.get("spawn_acoustic_marker", False),
                name="scenario.spawn_acoustic_marker",
            )
        ).lower(),
        "acoustic_marker_instance_file": _resolve_path(
            scenario.get("marker_instance_file", "")
        ),
        "acoustic_marker_descriptor_file": _resolve_path(
            scenario.get("marker_descriptor_file", "")
        ),
        "acoustic_marker_model_name": str(scenario.get("marker_model_name", "") or ""),
    }
    return values


def hull_vertices(mesh):
    """Return the convex horizontal hull of the simulator's binary STL mesh."""
    data = Path(_resolve_path(mesh)).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError("hull mesh must be a binary STL")
    raw_points = sorted(
        {
            struct.unpack_from("<ff", data, 84 + 50 * i + 12 + 12 * j)
            for i in range(count)
            for j in range(3)
        }
    )
    if len(raw_points) < 3:
        raise ValueError("hull mesh has fewer than three horizontal vertices")
    scale = max(
        max(point[axis] for point in raw_points)
        - min(point[axis] for point in raw_points)
        for axis in (0, 1)
    )
    tolerance = max(1e-9, scale * 1e-6)
    points = []
    for point in raw_points:
        if (
            not points
            or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1])
            > tolerance
        ):
            points.append(point)

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    lower, upper = [], []
    for seq, target in ((points, lower), (reversed(points), upper)):
        for point in seq:
            while (
                len(target) >= 2
                and cross(target[-2], target[-1], point) <= tolerance * scale
            ):
                target.pop()
            target.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError("hull mesh has no finite horizontal footprint")
    return [list(point) for point in hull]



def _finite_vector(raw, length, name, *, positive=False):
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    values = [float(value) for value in raw]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")
    if positive and not all(value > 0.0 for value in values):
        raise ValueError(f"{name} values must be positive")
    return values


def _write_box_world(output, boxes, *, world_name):
    """Write one ordinary SDF world from validated static box declarations."""
    normalized = []
    names = set()
    for index, raw in enumerate(boxes):
        box = dict(raw or {})
        name = str(box.get("name", "") or "").strip()
        if not name or name in names:
            raise ValueError("static obstacle names must be nonempty and unique")
        names.add(name)
        normalized.append(
            {
                **box,
                "name": name,
                "center": _finite_vector(
                    box.get("center"), 3, f"static_boxes[{index}].center"
                ),
                "size": _finite_vector(
                    box.get("size"),
                    3,
                    f"static_boxes[{index}].size",
                    positive=True,
                ),
            }
        )

    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name=str(world_name or "declared_scene"))
    physics = ET.SubElement(world, "physics", type="ode")
    for key, value in (
        ("max_step_size", ".004"),
        ("real_time_factor", "1"),
        ("real_time_update_rate", "250"),
    ):
        ET.SubElement(physics, key).text = value
    ET.SubElement(world, "gravity").text = "0 0 -9.80665"
    for uri in ("model://sun", "model://water_surface"):
        ET.SubElement(ET.SubElement(world, "include"), "uri").text = uri
    for box in normalized:
        model = ET.SubElement(world, "model", name=box["name"])
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = (
            " ".join(map(str, box["center"])) + " 0 0 0"
        )
        link = ET.SubElement(model, "link", name="body")
        for tag in ("collision", "visual"):
            geom = ET.SubElement(ET.SubElement(link, tag, name=tag), "geometry")
            ET.SubElement(ET.SubElement(geom, "box"), "size").text = " ".join(
                map(str, box["size"])
            )
    world_path = output / "environment.world"
    ET.ElementTree(sdf).write(world_path, encoding="utf-8", xml_declaration=True)
    entity_path = output / "entities.yaml"
    entity_path.write_text("entities: []\n", encoding="utf-8")
    return world_path, entity_path, normalized


def _materialize_static_obstacle_scene(declaration, output):
    """Resolve common geometry plus a named subset of optional obstacles."""
    hull = hull_vertices(declaration["hull_mesh"])
    required = [dict(item or {}) for item in declaration.get("static_boxes", ())]
    optional = [dict(item or {}) for item in declaration.get("optional_obstacles", ())]
    optional_by_name = {}
    for item in optional:
        name = str(item.get("name", "") or "").strip()
        if not name or name in optional_by_name:
            raise ValueError("optional obstacle names must be nonempty and unique")
        optional_by_name[name] = item
    enabled = [str(name) for name in declaration.get("enabled_obstacles", ())]
    if len(enabled) != len(set(enabled)):
        raise ValueError("enabled_obstacles contains duplicates")
    unknown = sorted(set(enabled) - set(optional_by_name))
    if unknown:
        raise ValueError("unknown enabled obstacle(s): " + ", ".join(unknown))
    selected = required + [optional_by_name[name] for name in enabled]

    spawn_raw = dict(declaration.get("spawn_pose", {}) or {})
    spawn = {
        "x": float(spawn_raw.get("x", 0.0)),
        "y": float(spawn_raw.get("y", 0.0)),
        "yaw_rad": float(spawn_raw.get("yaw_rad", 0.0)),
    }
    offset_raw = dict(declaration.get("world_offset", {}) or {})
    offset = {
        "x": float(offset_raw.get("x", 0.0)),
        "y": float(offset_raw.get("y", 0.0)),
    }
    if not all(math.isfinite(value) for value in (*spawn.values(), *offset.values())):
        raise ValueError("spawn_pose and world_offset must be finite")
    bounds = {
        name: float(value)
        for name, value in dict(declaration.get("map_bounds", {}) or {}).items()
    }
    if set(bounds) != {"x_min", "x_max", "y_min", "y_max"}:
        raise ValueError("map_bounds must define x_min, x_max, y_min and y_max")
    if not all(math.isfinite(value) for value in bounds.values()):
        raise ValueError("map_bounds must be finite")
    if bounds["x_min"] >= bounds["x_max"] or bounds["y_min"] >= bounds["y_max"]:
        raise ValueError("map_bounds minimums must be below maximums")

    world_path, entity_path, obstacles = _write_box_world(
        output,
        selected,
        world_name=declaration.get("world_name", "declared_scene"),
    )
    goal_raw = dict(declaration.get("goal", {}) or {})
    goal = {
        "x": float(goal_raw.get("x", 0.0)),
        "y": float(goal_raw.get("y", 0.0)),
        "yaw_rad": float(goal_raw.get("yaw_rad", 0.0)),
    }
    if not all(math.isfinite(value) for value in goal.values()):
        raise ValueError("goal must be finite")
    scenario = {
        "scenario_name": str(declaration.get("scenario_name", "declared_scene")),
        "world_file": str(world_path),
        "entity_file": str(entity_path),
        "map_bounds": bounds,
        "spawn_pose": spawn,
        "world_offset": offset,
        "obstacles": obstacles,
        "enabled_obstacles": enabled,
        "hull": hull,
        "goal": goal,
        "declaration": declaration,
    }
    path = output / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8")
    return str(path), scenario

def materialize_scenario(declaration, output_dir):
    """Build a declared wall scene from hull clearances and ordinary SDF boxes.

    Returned goal/geometry are experiment definitions, never a navigation map.
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    generator = str(declaration.get("generator", "") or "")
    if generator == "static_obstacle_scene":
        return _materialize_static_obstacle_scene(declaration, output)
    if generator != "wall_navigation":
        raise ValueError("unknown scenario generator: " + generator)
    hull = hull_vertices(declaration["hull_mesh"])
    clearance = float(declaration["clearance"])
    if not math.isfinite(clearance) or clearance < 0:
        raise ValueError("hull clearance must be finite and nonnegative")
    outward = {"front": (-1, 0), "rear": (1, 0), "right": (0, 1), "left": (0, -1)}
    thickness = float(declaration.get("wall_thickness", 0.4))
    length = float(declaration.get("wall_length", 40))
    height = float(declaration.get("wall_height", 2))
    walls = []
    for side in declaration["walls"]:
        nx, ny = outward[side]
        boundary = min(x * nx + y * ny for x, y in hull) - clearance
        center = [
            (boundary - thickness / 2) * nx,
            (boundary - thickness / 2) * ny,
            height / 2,
        ]
        walls.append(
            {
                "name": side,
                "normal": [nx, ny],
                "boundary": boundary,
                "center": center,
                "size": [
                    thickness if nx else length,
                    length if nx else thickness,
                    height,
                ],
            }
        )
    normals = [wall["normal"] for wall in walls]
    direction = [sum(n[j] for n in normals) for j in range(2)]
    maneuver = declaration["maneuver"]
    if maneuver == "follow":
        x, y = declaration["follow_delta"]
        yaw = 0.0
    elif maneuver == "turn":
        distance = float(declaration.get("turn_displacement", 1))
        x, y = [distance * v for v in direction]
        yaw = math.pi / 2
    elif maneuver == "escape":
        x = y = 0.0
        yaw = math.atan2(direction[1], direction[0])
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = [(c * px - s * py, s * px + c * py) for px, py in hull]
        for wall in walls:
            nx, ny = wall["normal"]
            shift = (
                wall["boundary"]
                + float(declaration.get("escape_clearance", 5))
                - min(px * nx + py * ny for px, py in rotated)
            )
            x += nx * shift
            y += ny * shift
    else:
        raise ValueError("unknown maneuver: " + str(maneuver))
    world_path, entity_path, _ = _write_box_world(
        output,
        walls + list(declaration.get("boxes", [])),
        world_name="declared_scene",
    )
    scenario = {
        "scenario_name": "declared_scene",
        "world_file": str(world_path),
        "entity_file": str(entity_path),
        "spawn_pose": {"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        "world_offset": {"x": 0.0, "y": 0.0},
        "walls": walls,
        "hull": hull,
        "goal": {"x": float(x), "y": float(y), "yaw_rad": float(yaw)},
        "declaration": declaration,
    }
    path = output / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8")
    return str(path), scenario


def scenario_names() -> Tuple[str, ...]:
    """Return every configured scenario name in deterministic order."""
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenarios = dict(index.get("scenarios", {}) or {})
    if not scenarios:
        raise ValueError(f"scenario index is empty: {SCENARIO_INDEX_PATH}")
    return tuple(sorted(str(name) for name in scenarios))


def scenario_launch_values(scenario: str) -> Dict[str, str]:
    """Resolve one coherent, validated launch bundle for ``scenario``."""
    index = _load_yaml(SCENARIO_INDEX_PATH)
    scenario_file = _scenario_file(index, scenario)
    if not scenario_file.is_file():
        raise FileNotFoundError(f"scenario configuration is missing: {scenario_file}")
    values = _launch_values(_load_yaml(scenario_file), scenario_file)
    required_files = {
        "sim_world_file": values["sim_world_file"],
        "map_entities_file": values["map_entities_file"],
    }
    if values["spawn_acoustic_marker"] == "true":
        required_files.update(
            {
                "acoustic_marker_instance_file": values[
                    "acoustic_marker_instance_file"
                ],
                "acoustic_marker_descriptor_file": values[
                    "acoustic_marker_descriptor_file"
                ],
            }
        )
        if not values["acoustic_marker_model_name"].strip():
            raise ValueError(
                f"scenario {scenario!r} enables a marker without a model name"
            )
    for field, raw_path in required_files.items():
        if not raw_path or not Path(raw_path).is_file():
            raise FileNotFoundError(
                f"scenario {scenario!r} has missing {field}: {raw_path or '<empty>'}"
            )
    return values
