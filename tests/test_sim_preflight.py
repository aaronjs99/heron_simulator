#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


_ORIG_MODULES = {name: sys.modules.get(name) for name in ("rospy", "gazebo_msgs.srv")}

mock_rospy = MagicMock()
mock_gazebo_srvs = MagicMock()
mock_gazebo_srvs.GetWorldProperties = object

sys.modules["rospy"] = mock_rospy
sys.modules["gazebo_msgs.srv"] = mock_gazebo_srvs

TEST_DIR = os.path.dirname(__file__)
SCRIPT_DIR = os.path.abspath(os.path.join(TEST_DIR, "..", "scripts", "tools"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import sim_preflight  # noqa: E402

for _name, _module in _ORIG_MODULES.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


class SimPreflightTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = mock_rospy
        sys.modules["gazebo_msgs.srv"] = mock_gazebo_srvs
        importlib.reload(sim_preflight)
        mock_rospy.reset_mock()

    def tearDown(self):
        for _name, _module in _ORIG_MODULES.items():
            if _module is None:
                sys.modules.pop(_name, None)
            else:
                sys.modules[_name] = _module

    def test_port_in_use_closes_socket(self):
        socket_instance = MagicMock()
        socket_instance.connect_ex.return_value = 0

        with patch.object(sim_preflight.socket, "socket", return_value=socket_instance):
            self.assertTrue(sim_preflight.port_in_use(11345))

        socket_instance.settimeout.assert_called_once_with(0.15)
        socket_instance.close.assert_called_once()

    def test_main_exits_when_ports_are_busy(self):
        params = {
            "~gazebo_master_port": 11345,
            "~allow_existing_gazebo": False,
            "~required_free_ports": [8080],
        }
        mock_rospy.get_param.side_effect = lambda name, default=None: params.get(
            name, default
        )
        mock_rospy.ServiceProxy.return_value = lambda: SimpleNamespace(
            model_names=["heron"]
        )

        with patch.object(
            sim_preflight,
            "port_in_use",
            side_effect=lambda port: int(port) in {11345, 8080},
        ):
            with patch.object(
                sim_preflight.sys, "exit", side_effect=SystemExit(1)
            ) as exit_mock:
                with self.assertRaises(SystemExit):
                    sim_preflight.main()

        exit_mock.assert_called_once_with(1)
        self.assertTrue(mock_rospy.logerr.called)

    def test_main_spins_when_environment_is_clean(self):
        params = {
            "~gazebo_master_port": 11345,
            "~allow_existing_gazebo": False,
            "~required_free_ports": [8070, 8080],
        }
        mock_rospy.get_param.side_effect = lambda name, default=None: params.get(
            name, default
        )

        with patch.object(sim_preflight, "port_in_use", return_value=False):
            sim_preflight.main()

        mock_rospy.spin.assert_called_once()
        mock_rospy.loginfo.assert_any_call("[sim_preflight] Environment looks clean.")


if __name__ == "__main__":
    unittest.main()
