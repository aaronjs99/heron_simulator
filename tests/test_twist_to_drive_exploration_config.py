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
    def test_exploration_mixer_profile_is_minimal_and_linear(self):
        with open(EXPLORATION_MIXER_CONFIG, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        self.assertEqual(data["max_linear"], 1.00)
        self.assertEqual(data["max_angular"], 0.24)
        self.assertEqual(data["yaw_mix"], 1.00)
        self.assertTrue(data["shared_drive_normalization"])
        self.assertEqual(data["rate"], 30.0)
        self.assertEqual(data["timeout"], 0.80)

        removed = {
            "deadband",
            "floor",
            "alpha",
            "ramp_down",
            "sign_hysteresis",
            "sign_dwell",
            "low_speed_turn_mix",
            "low_speed_turn_v_thresh",
            "no_reverse_v_thresh",
            "apply_input_shaping",
            "apply_sign_hysteresis",
            "wheelbase",
            "mix_gain",
        }
        self.assertTrue(removed.isdisjoint(data.keys()))


if __name__ == "__main__":
    unittest.main()
