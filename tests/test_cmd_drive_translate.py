#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
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

    def test_idle_axis_snaps_to_zero_inside_deadband(self):
        self.node.command_deadband = 0.03
        self.assertEqual(self.node.snap_idle_axis(0.02, 0.0), 0.0)
        self.assertAlmostEqual(self.node.snap_idle_axis(0.05, 0.0), 0.05)
        self.assertAlmostEqual(self.node.snap_idle_axis(0.02, 0.1), 0.02)


if __name__ == "__main__":
    unittest.main()
