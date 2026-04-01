#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
BRINGUP_LAUNCH = os.path.abspath(
    os.path.join(REPO_ROOT, "..", "slam_grande", "launch", "bringup.launch")
)


class SimulationLaunchTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(BRINGUP_LAUNCH).getroot()

    def test_simulation_launch_uses_teb_overlay_arg(self):
        mariner_group = self.root.find(
            "group[@if='$(arg use_mariner)']/group[@if=\"$(eval arg('mode') == 'sim')\"]"
        )
        self.assertIsNotNone(mariner_group)
        include = mariner_group.find(
            "include[@file='$(find mariner)/launch/move_base.launch']"
        )
        self.assertIsNotNone(include)
        args = {
            elem.attrib["name"]: elem.attrib["value"] for elem in include.findall("arg")
        }
        self.assertEqual(
            args["teb_local_planner_overlay_config"],
            "$(find mariner)/config/teb_local_planner_sim.yaml",
        )
        self.assertEqual(
            args["global_costmap_config"],
            "$(arg global_costmap_config)",
        )
        self.assertNotIn("teb_local_planner_config", args)
        self.assertEqual(
            args["local_costmap_overlay_config"],
            "$(find mariner)/config/local_costmap_sim.yaml",
        )

    def test_bringup_exposes_simulation_runtime_args(self):
        args = {elem.attrib["name"]: elem.attrib for elem in self.root.findall("arg")}
        for name in (
            "use_rviz",
            "rviz_config",
            "use_web_viz",
            "record_bags",
            "bag_output_dir",
            "bag_prefix",
            "spawn_inspection_models",
            "inspection_spawn_delay_sec",
            "inspection_spawn_hold_open",
            "build_map",
            "map_builder",
            "run_map",
            "map_file",
            "map_output_dir",
            "map_name",
            "map_anchors_file",
            "semantic_sim_fallback_enabled",
            "semantic_sim_fallback_file",
            "semantic_sim_fallback_timeout_sec",
            "semantic_sim_fallback_min_entities",
            "x",
            "y",
            "z",
            "yaw",
        ):
            self.assertIn(name, args)
        self.assertEqual(
            args["semantic_sim_fallback_enabled"]["default"],
            "$(eval 'true' if arg('mode') == 'sim' else 'false')",
        )
        self.assertEqual(
            args["semantic_sim_fallback_file"]["default"],
            "$(arg map_anchors_file)",
        )
        self.assertIn("anchors_sim.yaml", args["map_anchors_file"]["default"])
        self.assertEqual(
            args["run_map"]["default"],
            "$(eval 'true' if arg('mode') == 'sim' else 'false')",
        )
        self.assertIn("maps/simulation.yaml", args["map_file"]["default"])
        self.assertEqual(args["record_bags"]["default"], "true")
        self.assertEqual(args["inspection_spawn_delay_sec"]["default"], "15.0")

    def test_simulation_launch_wires_saved_map_and_runtime_surface(self):
        mariner_group = self.root.find(
            "group[@if='$(arg use_mariner)']/group[@if=\"$(eval arg('mode') == 'sim')\"]"
        )
        self.assertIsNotNone(mariner_group)

        include = mariner_group.find(
            "include[@file='$(find mariner)/launch/move_base.launch']"
        )
        self.assertIsNotNone(include)
        args = {
            elem.attrib["name"]: elem.attrib["value"] for elem in include.findall("arg")
        }
        self.assertEqual(args["use_map_server"], "$(arg run_map)")
        self.assertEqual(args["map_file"], "$(arg map_file)")

        relay = mariner_group.find("node[@name='move_base_status_relay']")
        self.assertIsNotNone(relay)
        self.assertEqual(relay.attrib["pkg"], "mariner")
        self.assertEqual(relay.attrib["type"], "goal_status_relay.py")

        sensor_map_builder = self.root.find("node[@name='build_sensor_nav_map_sim']")
        self.assertIsNotNone(sensor_map_builder)
        self.assertEqual(sensor_map_builder.attrib["pkg"], "mariner")
        self.assertEqual(
            sensor_map_builder.attrib["type"], "generate_map_from_pointcloud.py"
        )

        sim_group = self.root.find("group[@if=\"$(eval arg('mode') == 'sim')\"]")
        self.assertIsNotNone(sim_group)

        surface_include = None
        for group in self.root.findall("group[@if=\"$(eval arg('mode') == 'sim')\"]"):
            surface_include = group.find(
                "include[@file='$(find slam_grande)/launch/include/operator_surface.launch']"
            )
            if surface_include is not None:
                break
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
        self.assertEqual(
            surface_args["semantic_sim_fallback_enabled"],
            "$(arg semantic_sim_fallback_enabled)",
        )
        self.assertEqual(
            surface_args["semantic_sim_fallback_file"],
            "$(arg semantic_sim_fallback_file)",
        )
        self.assertEqual(surface_args["sim_mode"], "true")
        self.assertEqual(surface_args["sim_home_x"], "$(arg x)")
        self.assertEqual(surface_args["sim_home_y"], "$(arg y)")
        self.assertEqual(surface_args["sim_home_z"], "$(arg z)")
        self.assertEqual(surface_args["sim_home_yaw"], "$(arg yaw)")

        world_include = sim_group.find(
            "include[@file='$(find heron_simulator)/launch/heron_world.launch']"
        )
        self.assertIsNotNone(world_include)
        spawner = sim_group.find("node[@name='spawn_inspection_models']")
        self.assertIsNotNone(spawner)
        self.assertEqual(spawner.attrib["if"], "$(arg spawn_inspection_models)")
        self.assertNotIn("required", spawner.attrib)

    def test_simulation_launch_enables_dlio_when_building_sensor_map(self):
        sim_group = self.root.find("group[@if=\"$(eval arg('mode') == 'sim')\"]")
        self.assertIsNotNone(sim_group)
        dlio_group = sim_group.find(
            "group[@if=\"$(eval str(arg('use_dlio')).lower() == 'true' or str(arg('build_map')).lower() == 'true')\"]"
        )
        self.assertIsNotNone(dlio_group)


if __name__ == "__main__":
    unittest.main()
