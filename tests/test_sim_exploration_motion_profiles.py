#!/usr/bin/env python3
import unittest
from pathlib import Path

import yaml


class TestSimExplorationMotionProfiles(unittest.TestCase):
    def _load_yaml(self, relpath):
        cfg_path = Path(__file__).resolve().parents[1] / "config" / relpath
        with cfg_path.open("r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_exploration_mixer_is_more_direct_than_base_profile(self):
        base_cfg = self._load_yaml("twist_to_drive.yaml")
        exp_cfg = self._load_yaml("twist_to_drive_exploration.yaml")

        self.assertGreaterEqual(float(exp_cfg["max_linear"]), 1.00)
        self.assertGreater(float(exp_cfg["mix_gain"]), float(base_cfg["mix_gain"]))
        self.assertGreater(float(exp_cfg["alpha"]), float(base_cfg["alpha"]))
        self.assertLessEqual(float(exp_cfg["no_reverse_v_thresh"]), 0.01)
        self.assertLess(float(exp_cfg["low_speed_turn_mix"]), float(base_cfg["low_speed_turn_mix"]))

    def test_exploration_thruster_dynamics_reduces_filter_lag(self):
        base_cfg = self._load_yaml("thruster_dynamics.yaml")
        exp_cfg = self._load_yaml("thruster_dynamics_exploration.yaml")

        self.assertGreater(float(exp_cfg["max_delta_per_sec"]), float(base_cfg["max_delta_per_sec"]))
        self.assertLess(float(exp_cfg["time_constant_up"]), float(base_cfg["time_constant_up"]))
        self.assertLess(float(exp_cfg["time_constant_down"]), float(base_cfg["time_constant_down"]))
        self.assertGreater(float(exp_cfg["left_scale"]), float(base_cfg["left_scale"]))
        self.assertGreater(float(exp_cfg["right_scale"]), float(base_cfg["right_scale"]))


if __name__ == "__main__":
    unittest.main()
