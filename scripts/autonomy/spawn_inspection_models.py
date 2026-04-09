#!/usr/bin/env python3
"""Gazebo Prop Spawner: Dynamic World Population.
----------------------------------------------

Procedurally generates and spawns SDF models into the Gazebo world based on
the semantic map definition in `slam_grande/data/anchors.yaml`.

This allows the simulation environment to remain strictly synchronized with the
Cognitive Map (ORACLE), ensuring that every "Inspection Target" (Pillar, Pipe, Dock)
that the LLM knows about actually physically exists in the simulation.
"""
import rospy
from gazebo_msgs.srv import GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import os
import rospkg


def _make_service_proxy(service_name, service_type, persistent=True):
    proxy = rospy.ServiceProxy(service_name, service_type, persistent=persistent)
    return proxy


def _call_spawn_model(proxy, anchor_id, sdf, pose, retries=3):
    last_error = None
    shutdown_requested = False
    for attempt in range(1, retries + 1):
        if rospy.is_shutdown():
            shutdown_requested = True
            break
        try:
            return proxy(anchor_id, sdf, "", pose, "world"), proxy
        except (rospy.ServiceException, rospy.ROSException) as e:
            last_error = e
            rospy.logwarn(
                "  Spawn call for %s failed on attempt %d/%d: %s",
                anchor_id,
                attempt,
                retries,
                e,
            )
            try:
                proxy.close()
            except Exception:
                pass
            if attempt < retries:
                rospy.sleep(0.25)
                rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
                proxy = _make_service_proxy("/gazebo/spawn_sdf_model", SpawnModel)
    if last_error is not None:
        raise last_error
    if shutdown_requested:
        raise rospy.ROSInterruptException("ROS shutdown requested before spawn call")
    raise rospy.ROSException(f"Failed to spawn {anchor_id}: unknown spawn_model failure")


def make_sdf(name, sim_data):
    """Generate SDF string from detailed sim metadata."""
    geom_type = sim_data.get("geometry")
    visual = sim_data.get("visual", {})
    collision = sim_data.get("collision", {})

    def get_geom_xml(props, gtype):
        if gtype == "box":
            s = props.get("size", [1, 1, 1])
            return f"<box><size>{s[0]} {s[1]} {s[2]}</size></box>"
        elif gtype == "cylinder":
            r = float(props.get("radius", 0.5))
            l = float(props.get("length", 1.0))
            return f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"
        return ""

    def describe_geom(props, gtype):
        if gtype == "box":
            s = props.get("size", [1, 1, 1])
            return f"{gtype} size={s}"
        if gtype == "cylinder":
            r = float(props.get("radius", 0.5))
            l = float(props.get("length", 1.0))
            return f"{gtype} r={r:.2f} l={l:.2f}"
        return gtype

    vis_desc = describe_geom(visual, geom_type)
    col_desc = describe_geom(collision, geom_type)
    if vis_desc == col_desc:
        rospy.loginfo(f"  > {name} {vis_desc}")
    else:
        rospy.loginfo(f"  > {name} visual {vis_desc}")
        rospy.loginfo(f"  > {name} collision {col_desc}")

    vis_xml = get_geom_xml(visual, geom_type)
    col_xml = get_geom_xml(collision, geom_type)

    mat = visual.get("material", {})
    amb = mat.get("ambient", [0.5, 0.5, 0.5, 1])
    dif = mat.get("diffuse", [0.6, 0.6, 0.6, 1])

    # Format colors as space-separated strings
    a_str = f"{amb[0]} {amb[1]} {amb[2]} {amb[3]}"
    d_str = f"{dif[0]} {dif[1]} {dif[2]} {dif[3]}"

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
 <model name="{name}">
  <static>true</static>
  <link name="link">
   <visual name="visual">
    <geometry>{vis_xml}</geometry>
    <material>
     <ambient>{a_str}</ambient>
     <diffuse>{d_str}</diffuse>
    </material>
   </visual>
   <collision name="collision">
    <geometry>{col_xml}</geometry>
   </collision>
  </link>
 </model>
</sdf>
"""


def load_anchors(anchor_file=""):
    """Load anchors from an explicit file or the slam_grande default."""
    path = str(anchor_file or "").strip()
    if not path:
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path("slam_grande"), "data", "anchors.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)


def apply_world_offset(data, dx=0.0, dy=0.0):
    """Apply a rigid XY translation to all anchor poses in the loaded YAML."""
    if not isinstance(data, dict) or (
        abs(float(dx)) < 1e-9 and abs(float(dy)) < 1e-9
    ):
        return data
    anchors = list(data.get("anchors", []) or [])
    if not anchors:
        return data
    shifted = dict(data)
    shifted_anchors = []
    for item in anchors:
        if not isinstance(item, dict):
            shifted_anchors.append(item)
            continue
        entry = dict(item)
        pose = dict(entry.get("pose", {}) or {})
        position = dict(pose.get("position", {}) or {})
        position["x"] = float(position.get("x", entry.get("x", 0.0))) + float(dx)
        position["y"] = float(position.get("y", entry.get("y", 0.0))) + float(dy)
        pose["position"] = position
        entry["pose"] = pose
        shifted_anchors.append(entry)
    shifted["anchors"] = shifted_anchors
    return shifted


def main():
    rospy.init_node("spawn_inspection_models", anonymous=True)
    hold_open = rospy.get_param("~hold_open", True)
    anchor_file = str(rospy.get_param("~anchor_file", "") or "").strip()
    world_offset_x = float(rospy.get_param("~world_offset_x", 0.0))
    world_offset_y = float(rospy.get_param("~world_offset_y", 0.0))

    rospy.loginfo("Loading anchors from YAML...")
    try:
        data = load_anchors(anchor_file)
        data = apply_world_offset(data, dx=world_offset_x, dy=world_offset_y)
    except Exception as e:
        rospy.logerr(f"Failed to load anchors YAML ({anchor_file or 'default'}): {e}")
        return

    rospy.loginfo("Waiting for gazebo/spawn_sdf_model service...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    spawn_model = _make_service_proxy("/gazebo/spawn_sdf_model", SpawnModel)
    get_world_properties = None
    existing_models = set()
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5.0)
        get_world_properties = _make_service_proxy(
            "/gazebo/get_world_properties", GetWorldProperties
        )
        existing_models = set(get_world_properties().model_names)
        rospy.loginfo("Existing Gazebo models: %d", len(existing_models))
    except (rospy.ROSException, rospy.ServiceException) as e:
        rospy.logwarn("Could not query existing Gazebo models: %s", e)

    rospy.loginfo("Spawning inspection models...")
    failures = []

    # Support both hierarchical dictionary and list-based formats
    anchors_list = []
    if isinstance(data, dict):
        if "anchors" in data and isinstance(data["anchors"], list):
            anchors_list = data["anchors"]
        else:
            # Fallback to old dict format where keys are IDs
            for aid, props in data.items():
                if isinstance(props, dict):
                    props["id"] = aid
                    anchors_list.append(props)

    for props in anchors_list:
        anchor_id = props.get("id")
        if not anchor_id:
            continue

        # Only spawn items that have 'sim' metadata
        if "sim" not in props:
            continue

        sim = props["sim"]

        # Robust pose extraction
        try:
            if "pose" in props:
                pose_data = props["pose"]
                p = pose_data["position"]
                o = pose_data["orientation"]
            else:
                # Handle flat format or other variations if necessary
                p = props.get("position", {"x": 0, "y": 0, "z": 0})
                o = props.get("orientation", {"x": 0, "y": 0, "z": 0, "w": 1})
                if isinstance(p, list):  # handle [x,y,z] format
                    p = {"x": p[0], "y": p[1], "z": p[2]}

            pose = Pose()
            pose.position = Point(
                float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0))
            )
            pose.orientation = Quaternion(
                float(o.get("x", 0)),
                float(o.get("y", 0)),
                float(o.get("z", 0)),
                float(o.get("w", 1)),
            )
        except Exception as e:
            rospy.logwarn(f"  Skipping {anchor_id} due to invalid pose data: {e}")
            continue

        # Generate SDF
        # Now passing the whole 'sim' dictionary
        sdf = make_sdf(anchor_id, sim)

        if anchor_id in existing_models:
            rospy.loginfo(f"  Skipping {anchor_id}: model already exists.")
            continue

        try:
            resp, spawn_model = _call_spawn_model(spawn_model, anchor_id, sdf, pose)
            if resp.success:
                existing_models.add(anchor_id)
                rospy.loginfo(
                    f"  Spawned {anchor_id} at ({p['x']}, {p['y']}, {p['z']})"
                )
            else:
                failures.append((anchor_id, resp.status_message))
                rospy.logwarn(f"  Failed to spawn {anchor_id}: {resp.status_message}")
        except (rospy.ServiceException, rospy.ROSException) as e:
            failures.append((anchor_id, str(e)))
            rospy.logerr(f"  Service call failed for {anchor_id}: {e}")

    try:
        spawn_model.close()
    except Exception:
        pass
    try:
        get_world_properties.close()
    except Exception:
        pass

    if failures:
        rospy.logerr(
            "Inspection models completed with %d failures: %s",
            len(failures),
            ", ".join(anchor_id for anchor_id, _ in failures[:10]),
        )
        raise SystemExit(1)
    else:
        rospy.loginfo("Inspection models spawned successfully!")
        if hold_open:
            rospy.loginfo("Inspection model spawner standing by.")
            rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
