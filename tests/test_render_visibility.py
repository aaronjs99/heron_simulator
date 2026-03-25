#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
PLUGIN_CPP = os.path.join(REPO_ROOT, "src", "PathVisualizerPlugin.cpp")
HERON_SIM_XACRO = os.path.join(REPO_ROOT, "urdf", "heron_sim.urdf.xacro")
IG_HANDLE_XACRO = os.path.join(REPO_ROOT, "urdf", "ig_handle_sensors.urdf.xacro")


class RenderVisibilityTests(unittest.TestCase):
    def test_path_visualizer_is_gui_only(self):
        with open(PLUGIN_CPP, "r", encoding="utf-8") as handle:
            source = handle.read()

        self.assertIn(
            "this->local_path_object_->setVisibilityFlags(GZ_VISIBILITY_GUI);",
            source,
        )
        self.assertIn(
            "this->global_path_object_->setVisibilityFlags(GZ_VISIBILITY_GUI);",
            source,
        )

    def test_camera_visibility_masks_exclude_gui_flag(self):
        for path in (HERON_SIM_XACRO, IG_HANDLE_XACRO):
            tree = ET.parse(path)
            root = tree.getroot()
            cameras = root.findall(".//camera")
            self.assertGreaterEqual(len(cameras), 2, path)
            for camera in cameras:
                visibility_mask = camera.find("visibility_mask")
                self.assertIsNotNone(visibility_mask, path)
                self.assertEqual(visibility_mask.text.strip(), "4294967294")


if __name__ == "__main__":
    unittest.main()
