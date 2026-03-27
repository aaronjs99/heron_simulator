#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SIM_LAUNCH = os.path.join(REPO_ROOT, "launch", "run.launch")
SIM_SHIM = os.path.join(REPO_ROOT, "launch", "simulation_full.launch")


class SimulationLaunchTests(unittest.TestCase):
    def setUp(self):
        self.tree = ET.parse(SIM_LAUNCH)
        self.root = self.tree.getroot()

    def test_simulation_launch_uses_teb_overlay_arg(self):
        include = None
        for elem in self.root.findall("group/include"):
            if elem.attrib.get("file") == "$(find mariner)/launch/move_base.launch":
                include = elem
                break

        self.assertIsNotNone(include)
        args = {
            elem.attrib["name"]: elem.attrib["value"] for elem in include.findall("arg")
        }
        self.assertEqual(
            args["teb_local_planner_overlay_config"],
            "$(find mariner)/config/teb_local_planner_sim.yaml",
        )
        self.assertNotIn("teb_local_planner_config", args)
        self.assertEqual(
            args["local_costmap_overlay_config"],
            "$(find mariner)/config/local_costmap_sim.yaml",
        )

    def test_simulation_launch_exposes_static_map_and_rviz_args(self):
        args = {elem.attrib["name"]: elem.attrib for elem in self.root.findall("arg")}
        for name in (
            "use_rviz",
            "rviz_config",
            "use_web_viz",
            "record_bags",
            "bag_output_dir",
            "bag_prefix",
            "build_map",
            "map_builder",
            "run_map",
            "map_file",
            "map_output_dir",
            "map_name",
            "map_anchors_file",
        ):
            self.assertIn(name, args)
        self.assertEqual(
            args["map_anchors_file"]["default"],
            "$(find slam_grande)/data/anchors_sim.yaml",
        )
        self.assertEqual(args["run_map"]["default"], "true")
        self.assertEqual(
            args["map_file"]["default"],
            "$(find mariner)/maps/simulation.yaml",
        )
        self.assertEqual(args["record_bags"]["default"], "true")

    def test_simulation_launch_wires_saved_map_and_rviz_include(self):
        include = None
        for elem in self.root.findall("group/include"):
            if elem.attrib.get("file") == "$(find mariner)/launch/move_base.launch":
                include = elem
                break

        self.assertIsNotNone(include)
        args = {
            elem.attrib["name"]: elem.attrib["value"] for elem in include.findall("arg")
        }
        self.assertEqual(args["use_map_server"], "$(arg run_map)")
        self.assertEqual(args["map_file"], "$(arg map_file)")

        relay = self.root.find("group/node[@name='move_base_status_relay']")
        self.assertIsNotNone(relay)
        self.assertEqual(relay.attrib["pkg"], "mariner")
        self.assertEqual(relay.attrib["type"], "goal_status_relay.py")

        sensor_map_builder = self.root.find("node[@name='build_sensor_nav_map']")
        self.assertIsNotNone(sensor_map_builder)
        self.assertEqual(sensor_map_builder.attrib["pkg"], "mariner")
        self.assertEqual(
            sensor_map_builder.attrib["type"], "generate_map_from_pointcloud.py"
        )

        surface_include = self.root.find(
            "include[@file='$(find slam_grande)/launch/include/operator_surface.launch']"
        )
        self.assertIsNotNone(surface_include)
        surface_args = {
            elem.attrib["name"]: elem.attrib["value"]
            for elem in surface_include.findall("arg")
        }
        self.assertEqual(surface_args["use_rviz"], "$(arg use_rviz)")
        self.assertEqual(surface_args["rviz_config"], "$(arg rviz_config)")
        self.assertEqual(surface_args["record_bags"], "$(arg record_bags)")
        self.assertEqual(surface_args["bag_output_dir"], "$(arg bag_output_dir)")
        self.assertEqual(surface_args["bag_prefix"], "$(arg bag_prefix)")

    def test_simulation_full_launch_is_a_compatibility_shim(self):
        root = ET.parse(SIM_SHIM).getroot()
        include = root.find("include")
        self.assertIsNotNone(include)
        self.assertEqual(include.attrib["file"], "$(find heron_simulator)/launch/run.launch")
        self.assertEqual(include.attrib["pass_all_args"], "true")

    def test_simulation_launch_enables_dlio_when_building_sensor_map(self):
        group = None
        for elem in self.root.findall("group"):
            if (
                elem.attrib.get("if")
                == "$(eval str(arg('use_dlio')).lower() == 'true' or str(arg('build_map')).lower() == 'true')"
            ):
                group = elem
                break
        self.assertIsNotNone(group)


if __name__ == "__main__":
    unittest.main()
