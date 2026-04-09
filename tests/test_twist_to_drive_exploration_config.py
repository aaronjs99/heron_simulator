#!/usr/bin/env python3
import os
import unittest

import yaml


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
EXPLORATION_MIXER_CONFIG = os.path.join(
    REPO_ROOT, "config", "twist_to_drive_exploration.yaml"
)


class TwistToDriveExplorationConfigTests(unittest.TestCase):
    def test_exploration_mixer_profile_caps_close_quarters_aggression(self):
        with open(EXPLORATION_MIXER_CONFIG, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        self.assertEqual(data["max_linear"], 0.75)
        self.assertEqual(data["max_angular"], 0.20)
        self.assertEqual(data["alpha"], 0.22)
        self.assertEqual(data["ramp_down"], 0.35)
        self.assertEqual(data["low_speed_turn_mix"], 0.30)
        self.assertEqual(data["low_speed_turn_v_thresh"], 0.20)


if __name__ == "__main__":
    unittest.main()
