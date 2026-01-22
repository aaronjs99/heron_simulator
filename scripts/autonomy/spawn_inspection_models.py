#!/usr/bin/env python3
"""Gazebo Prop Spawner: Dynamic World Population.
----------------------------------------------

Procedurally generates and spawns SDF models into the Gazebo world based on
the semantic map definition in `oracle/data/anchors.yaml`.

This allows the simulation environment to remain strictly synchronized with the
Cognitive Map (ORACLE), ensuring that every "Inspection Target" (Pillar, Pipe, Dock)
that the LLM knows about actually physically exists in the simulation.
"""
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import os
import rospkg


def make_sdf(name, sim_data):
    """Generate SDF string from detailed sim metadata."""
    geom_type = sim_data.get("geometry")
    visual = sim_data.get("visual", {})
    collision = sim_data.get("collision", {})

    def get_geom_xml(props, gtype):
        if gtype == "box":
            s = props.get("size", [1, 1, 1])
            rospy.loginfo(f"  > {name} {gtype} size={s}")
            return f"<box><size>{s[0]} {s[1]} {s[2]}</size></box>"
        elif gtype == "cylinder":
            r = float(props.get("radius", 0.5))
            l = float(props.get("length", 1.0))
            rospy.loginfo(f"  > {name} {gtype} r={r:.2f} l={l:.2f}")
            return f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"
        return ""

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


def load_anchors():
    """Load anchors from slam_grande package."""
    rospack = rospkg.RosPack()
    path = os.path.join(rospack.get_path("slam_grande"), "config", "anchors.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    rospy.init_node("spawn_inspection_models", anonymous=True)

    rospy.loginfo("Loading anchors from YAML...")
    try:
        data = load_anchors()
    except Exception as e:
        rospy.logerr(f"Failed to load anchors.yaml: {e}")
        return

    rospy.loginfo("Waiting for gazebo/spawn_sdf_model service...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    rospy.loginfo("Spawning inspection models...")

    for anchor_id, props in data.items():
        # Only spawn items that have 'sim' metadata
        if "sim" not in props:
            continue

        sim = props["sim"]
        pose_data = props["pose"]

        # Extract pose
        p = pose_data["position"]
        o = pose_data["orientation"]

        pose = Pose()
        pose.position = Point(p["x"], p["y"], p["z"])
        pose.orientation = Quaternion(o["x"], o["y"], o["z"], o["w"])

        # Generate SDF
        # Now passing the whole 'sim' dictionary
        sdf = make_sdf(anchor_id, sim)

        try:
            resp = spawn_model(anchor_id, sdf, "", pose, "world")
            if resp.success:
                rospy.loginfo(
                    f"  Spawned {anchor_id} at ({p['x']}, {p['y']}, {p['z']})"
                )
            else:
                rospy.logwarn(f"  Failed to spawn {anchor_id}: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"  Service call failed for {anchor_id}: {e}")

    rospy.loginfo("Inspection models spawned successfully!")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
