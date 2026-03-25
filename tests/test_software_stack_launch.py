#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SOFTWARE_STACK_LAUNCH = os.path.join(REPO_ROOT, "launch", "software_stack.launch")


class SoftwareStackLaunchTests(unittest.TestCase):
    def test_uses_fixed_type_move_base_status_relay(self):
        root = ET.parse(SOFTWARE_STACK_LAUNCH).getroot()
        relay = root.find("node[@name='move_base_status_relay']")
        self.assertIsNotNone(relay)
        self.assertEqual(relay.attrib["pkg"], "mariner")
        self.assertEqual(relay.attrib["type"], "goal_status_relay.py")


if __name__ == "__main__":
    unittest.main()
