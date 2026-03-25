#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SIM_LAUNCH = os.path.join(REPO_ROOT, "launch", "simulation_full.launch")


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
        args = {elem.attrib["name"]: elem.attrib["value"] for elem in include.findall("arg")}
        self.assertEqual(
            args["teb_local_planner_overlay_config"],
            "$(find mariner)/config/teb_local_planner_sim.yaml",
        )
        self.assertNotIn("teb_local_planner_config", args)
        self.assertEqual(
            args["local_costmap_overlay_config"],
            "$(find mariner)/config/local_costmap_sim.yaml",
        )


if __name__ == "__main__":
    unittest.main()
