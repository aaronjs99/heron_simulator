#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
import inspect
from unittest.mock import MagicMock


class FakeDuration:
    def __init__(self, secs):
        self.secs = float(secs)

    def to_sec(self):
        return self.secs


class FakeTime:
    def __init__(self, secs=0.0):
        self.secs = float(secs)

    def __sub__(self, other):
        return FakeDuration(self.secs - other.secs)

    def to_nsec(self):
        return int(self.secs * 1e9)


class FakeTimer:
    def shutdown(self):
        return None


_ORIG_MODULES = {
    name: sys.modules.get(name)
    for name in ("rospy", "geometry_msgs.msg", "heron_msgs.msg")
}

mock_rospy = MagicMock()
mock_geometry_msgs = MagicMock()
mock_heron_msgs = MagicMock()
mock_geometry_msgs.Wrench = MagicMock
mock_heron_msgs.Drive = MagicMock

sys.modules["rospy"] = mock_rospy
sys.modules["geometry_msgs.msg"] = mock_geometry_msgs
sys.modules["heron_msgs.msg"] = mock_heron_msgs

TEST_DIR = os.path.dirname(__file__)
SCRIPT_DIR = os.path.abspath(os.path.join(TEST_DIR, "..", "scripts", "control"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import cmd_drive_translate  # noqa: E402

for _name, _module in _ORIG_MODULES.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


class ThrusterTranslatorTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = mock_rospy
        sys.modules["geometry_msgs.msg"] = mock_geometry_msgs
        sys.modules["heron_msgs.msg"] = mock_heron_msgs
        importlib.reload(cmd_drive_translate)
        mock_rospy.Time.side_effect = lambda secs=0.0: FakeTime(secs)
        mock_rospy.Time.now.return_value = FakeTime(0.0)
        mock_rospy.Timer.return_value = FakeTimer()
        mock_rospy.get_param.side_effect = lambda name, default=None: default
        self.node = cmd_drive_translate.ThrusterTranslator()

    def tearDown(self):
        for _name, _module in _ORIG_MODULES.items():
            if _module is None:
                sys.modules.pop(_name, None)
            else:
                sys.modules[_name] = _module

    def test_shape_drive_zero_stays_zero(self):
        self.assertEqual(self.node.shape_drive(0.0, 1.0), 0.0)

    def test_shape_drive_scale_and_saturation(self):
        self.assertAlmostEqual(self.node.shape_drive(0.4, 1.25), 0.5, places=6)
        self.assertAlmostEqual(self.node.shape_drive(-0.5, 1.0), -0.5, places=6)
        self.assertAlmostEqual(self.node.shape_drive(0.9, 2.0), 1.0, places=6)
        self.assertAlmostEqual(self.node.shape_drive(-0.9, 2.0), -1.0, places=6)

    def test_piecewise_linear_thrust_mapping(self):
        self.node.max_fwd_thrust = 45.0
        self.node.max_bck_thrust = 25.0
        self.assertAlmostEqual(self.node.drive_to_thrust(1.0), 45.0, places=6)
        self.assertAlmostEqual(self.node.drive_to_thrust(0.2), 9.0, places=6)
        self.assertAlmostEqual(self.node.drive_to_thrust(-1.0), -25.0, places=6)
        self.assertAlmostEqual(self.node.drive_to_thrust(-0.2), -5.0, places=6)

    def test_stale_timeout_zeroes_targets_and_outputs(self):
        self.node.target_left = 0.4
        self.node.target_right = -0.4
        self.node.last_cmd_time = FakeTime(0.0)

        mock_rospy.Time.now.return_value = FakeTime(1.0)
        self.node.update(None)

        self.assertEqual(self.node.target_left, 0.0)
        self.assertEqual(self.node.target_right, 0.0)
        left_force = self.node.p_left.publish.call_args[0][0].force.x
        right_force = self.node.p_right.publish.call_args[0][0].force.x
        self.assertAlmostEqual(left_force, 0.0, places=6)
        self.assertAlmostEqual(right_force, 0.0, places=6)

    def test_translator_excludes_legacy_synthetic_actuator_shaping(self):
        source = inspect.getsource(cmd_drive_translate.ThrusterTranslator)
        self.assertNotIn("input_points", source)
        self.assertNotIn("output_thrust", source)
        self.assertNotIn("thrust_noise_stddev", source)
        self.assertNotIn("min_effective_drive", source)


if __name__ == "__main__":
    unittest.main()
