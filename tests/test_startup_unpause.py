#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
from unittest.mock import MagicMock, patch


_ORIG_MODULES = {name: sys.modules.get(name) for name in ("rospy", "std_srvs.srv")}

mock_rospy = MagicMock()
mock_std_srvs = MagicMock()
mock_std_srvs.Empty = object

sys.modules["rospy"] = mock_rospy
sys.modules["std_srvs.srv"] = mock_std_srvs

TEST_DIR = os.path.dirname(__file__)
SCRIPT_DIR = os.path.abspath(os.path.join(TEST_DIR, "..", "scripts", "tools"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import startup_unpause  # noqa: E402

for _name, _module in _ORIG_MODULES.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


class StartupUnpauseTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = mock_rospy
        sys.modules["std_srvs.srv"] = mock_std_srvs
        importlib.reload(startup_unpause)
        mock_rospy.reset_mock()

    def tearDown(self):
        for _name, _module in _ORIG_MODULES.items():
            if _module is None:
                sys.modules.pop(_name, None)
            else:
                sys.modules[_name] = _module

    def test_main_waits_then_calls_unpause_service(self):
        params = {
            "~delay_sec": 2.5,
            "~service": "/gazebo/unpause_physics",
        }
        mock_rospy.get_param.side_effect = lambda name, default=None: params.get(
            name, default
        )
        mock_rospy.is_shutdown.return_value = False
        service = MagicMock()
        mock_rospy.ServiceProxy.return_value = service

        with patch.object(startup_unpause.time, "sleep") as sleep_mock:
            startup_unpause.main()

        sleep_mock.assert_called_once_with(2.5)
        mock_rospy.wait_for_service.assert_called_once_with(
            "/gazebo/unpause_physics", timeout=15.0
        )
        service.assert_called_once_with()

    def test_main_returns_early_when_ros_is_shutting_down(self):
        params = {
            "~delay_sec": 1.0,
            "~service": "/gazebo/unpause_physics",
        }
        mock_rospy.get_param.side_effect = lambda name, default=None: params.get(
            name, default
        )
        mock_rospy.is_shutdown.return_value = True

        with patch.object(startup_unpause.time, "sleep"):
            startup_unpause.main()

        mock_rospy.wait_for_service.assert_not_called()
        mock_rospy.ServiceProxy.assert_not_called()


if __name__ == "__main__":
    unittest.main()
