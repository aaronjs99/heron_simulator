#!/usr/bin/env python3
import unittest
from pathlib import Path

import yaml


class TestSimExplorationMotionProfiles(unittest.TestCase):
    def _load_yaml(self, relpath):
        cfg_path = Path(__file__).resolve().parents[1] / "config" / relpath
        with cfg_path.open("r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_exploration_mixer_uses_minimal_linear_profile(self):
        exp_cfg = self._load_yaml("twist_to_drive_exploration.yaml")

        self.assertEqual(float(exp_cfg["max_linear"]), 1.00)
        self.assertEqual(float(exp_cfg["max_angular"]), 0.24)
        self.assertEqual(float(exp_cfg["yaw_mix"]), 1.00)
        self.assertTrue(bool(exp_cfg["shared_drive_normalization"]))
        self.assertEqual(float(exp_cfg["timeout"]), 0.80)

    def test_exploration_thruster_dynamics_matches_controller_thrust_model(self):
        exp_cfg = self._load_yaml("thruster_dynamics_exploration.yaml")

        self.assertEqual(
            set(exp_cfg.keys()),
            {
                "rate",
                "cmd_timeout",
                "max_fwd_thrust",
                "max_bck_thrust",
                "left_scale",
                "right_scale",
            },
        )
        self.assertEqual(float(exp_cfg["cmd_timeout"]), 0.80)
        self.assertEqual(float(exp_cfg["max_fwd_thrust"]), 45.0)
        self.assertEqual(float(exp_cfg["max_bck_thrust"]), 25.0)
        self.assertEqual(float(exp_cfg["left_scale"]), 1.0)
        self.assertEqual(float(exp_cfg["right_scale"]), 1.0)


if __name__ == "__main__":
    unittest.main()
