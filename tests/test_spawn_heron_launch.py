#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SPAWN_HERON_LAUNCH = os.path.join(REPO_ROOT, "launch", "spawn_heron.launch")


class SpawnHeronLaunchTests(unittest.TestCase):
    def test_launch_publishes_joint_states_for_robot_model(self):
        root = ET.parse(SPAWN_HERON_LAUNCH).getroot()
        joint_state_publisher = root.find(".//node[@name='joint_state_publisher']")
        self.assertIsNotNone(joint_state_publisher)
        self.assertEqual(joint_state_publisher.attrib["pkg"], "joint_state_publisher")
        self.assertEqual(joint_state_publisher.attrib["type"], "joint_state_publisher")


if __name__ == "__main__":
    unittest.main()
