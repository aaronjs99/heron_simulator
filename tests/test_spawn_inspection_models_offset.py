#!/usr/bin/env python3
import importlib.util
import os
import unittest


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "autonomy", "spawn_inspection_models.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("spawn_inspection_models", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


class SpawnInspectionModelsOffsetTests(unittest.TestCase):
    def test_apply_world_offset_translates_anchor_positions_rigidly(self):
        module = _load_module()
        anchors = {
            "anchors": [
                {
                    "id": "dock_a",
                    "pose": {
                        "position": {"x": -24.0, "y": 10.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                {
                    "id": "pile_a",
                    "pose": {
                        "position": {"x": -20.0, "y": 16.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
            ]
        }

        shifted = module.apply_world_offset(anchors, dx=24.0, dy=-10.0)
        shifted_positions = {
            item["id"]: item["pose"]["position"] for item in shifted["anchors"]
        }

        self.assertEqual(shifted_positions["dock_a"]["x"], 0.0)
        self.assertEqual(shifted_positions["dock_a"]["y"], 0.0)
        self.assertEqual(shifted_positions["pile_a"]["x"], 4.0)
        self.assertEqual(shifted_positions["pile_a"]["y"], 6.0)


if __name__ == "__main__":
    unittest.main()
