#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
from unittest.mock import MagicMock

_ORIG_MODULES = {name: sys.modules.get(name) for name in ("rospy", "rosgraph")}

sys.modules["rospy"] = MagicMock()
sys.modules["rosgraph"] = MagicMock()

TEST_DIR = os.path.dirname(__file__)
SCRIPT_DIR = os.path.abspath(os.path.join(TEST_DIR, "..", "scripts", "tools"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import topic_contract_guard  # noqa: E402

for _name, _module in _ORIG_MODULES.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


class TopicContractGuardTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(topic_contract_guard)
        self.contract = topic_contract_guard.load_topic_contract(
            {
                "topics": {
                    "pose_gt": "/pose_gt",
                    "odom": "/state/odometry",
                    "odom_filtered": "/state/odometry/filtered",
                    "cmd_vel_raw": "/cmd_vel_raw",
                    "cmd_vel": "/cmd_vel",
                    "cmd_drive": "/cmd_drive",
                    "thruster_left": "/thrusters/1/input",
                    "thruster_right": "/thrusters/0/input",
                    "nav_goal": "/mariner/final_pose",
                    "nav_goal_pose": "/mariner/goal",
                    "nav_goal_anchor": "/mariner/goal_anchor",
                    "nav_ok": "/mariner/status/nav_ok",
                    "nav_status": "/mariner/status/move_base",
                    "oracle_input": "/oracle/input",
                    "oracle_output": "/oracle/output",
                    "oracle_status": "/oracle/status",
                    "oracle_anchors": "/oracle/map/anchors",
                    "imu_data": "/sensors/imu/data",
                    "lidar_h_points": "/sensors/lidar/hori/points",
                    "sonar_scan": "/sensors/sonar/scan",
                    "camera_f1_image": "/sensors/camera/f1/image_raw",
                    "dlio_odom": "/state/dlio/odometry",
                    "dlio_path": "/state/dlio/path",
                },
                "types": {
                    "odometry": "nav_msgs/Odometry",
                    "twist": "geometry_msgs/Twist",
                    "drive": "heron_msgs/Drive",
                    "wrench": "geometry_msgs/Wrench",
                    "pose_stamped": "geometry_msgs/PoseStamped",
                    "string": "std_msgs/String",
                    "goal_status_array": "actionlib_msgs/GoalStatusArray",
                    "anchor_pose_array": "oracle/AnchorPoseArray",
                    "human_query_input": "oracle/HumanQueryInput",
                    "mission_status": "oracle/MissionStatus",
                    "imu": "sensor_msgs/Imu",
                    "pointcloud2": "sensor_msgs/PointCloud2",
                    "image": "sensor_msgs/Image",
                    "range": "sensor_msgs/Range",
                    "path": "nav_msgs/Path",
                },
                "forbidden_topics": {
                    "legacy_nav_ok": {
                        "topic": "/mariner/nav_ok",
                        "reason": "canonical nav_ok is /mariner/status/nav_ok",
                    }
                },
                "modules": {
                    "simulation_truth": {
                        "enabled": True,
                        "endpoints": {
                            "pose_gt": {
                                "topic_key": "pose_gt",
                                "type_key": "odometry",
                                "publishers": {"min": 1, "max": 1},
                            }
                        },
                    },
                    "state": {
                        "enabled": True,
                        "endpoints": {
                            "odom": {
                                "topic_key": "odom",
                                "type_key": "odometry",
                                "publishers": {"min": 1, "max": 1},
                            },
                            "odom_filtered": {
                                "topic_key": "odom_filtered",
                                "type_key": "odometry",
                                "publishers": {"min": 1, "max": 1},
                            },
                        },
                    },
                    "mariner": {
                        "enabled": True,
                        "endpoints": {
                            "cmd_vel_raw": {
                                "topic_key": "cmd_vel_raw",
                                "type_key": "twist",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "cmd_vel": {
                                "topic_key": "cmd_vel",
                                "type_key": "twist",
                                "publishers": {"max": 1},
                                "subscribers": {"min": 1},
                            },
                            "cmd_drive": {
                                "topic_key": "cmd_drive",
                                "type_key": "drive",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "thruster_left": {
                                "topic_key": "thruster_left",
                                "type_key": "wrench",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "thruster_right": {
                                "topic_key": "thruster_right",
                                "type_key": "wrench",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "nav_ok": {
                                "topic_key": "nav_ok",
                                "type_key": "string",
                                "publishers": {"min": 1, "max": 1},
                            },
                            "nav_status": {
                                "topic_key": "nav_status",
                                "type_key": "goal_status_array",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                        },
                    },
                    "goal_routing": {
                        "enabled": True,
                        "endpoints": {
                            "nav_goal": {
                                "topic_key": "nav_goal",
                                "type_key": "pose_stamped",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "nav_goal_pose": {
                                "topic_key": "nav_goal_pose",
                                "type_key": "pose_stamped",
                                "subscribers": {"min": 1},
                            },
                            "nav_goal_anchor": {
                                "topic_key": "nav_goal_anchor",
                                "type_key": "string",
                                "subscribers": {"min": 1},
                            },
                        },
                    },
                    "oracle": {
                        "enabled": True,
                        "endpoints": {
                            "input": {
                                "topic_key": "oracle_input",
                                "type_key": "human_query_input",
                            },
                            "output": {
                                "topic_key": "oracle_output",
                                "type_key": "string",
                            },
                            "status": {
                                "topic_key": "oracle_status",
                                "type_key": "mission_status",
                            },
                            "anchors": {
                                "topic_key": "oracle_anchors",
                                "type_key": "anchor_pose_array",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                        },
                    },
                    "ig_handle_sensors": {
                        "enabled": True,
                        "endpoints": {
                            "imu_data": {
                                "topic_key": "imu_data",
                                "type_key": "imu",
                                "publishers": {"min": 1, "max": 1},
                            },
                            "lidar_h_points": {
                                "topic_key": "lidar_h_points",
                                "type_key": "pointcloud2",
                                "publishers": {"min": 1, "max": 1},
                                "subscribers": {"min": 1},
                            },
                            "sonar_scan": {
                                "topic_key": "sonar_scan",
                                "type_key": "pointcloud2",
                                "publishers": {"min": 1, "max": 1},
                            },
                            "camera_f1_image": {
                                "topic_key": "camera_f1_image",
                                "type_key": "image",
                                "publishers": {"min": 1, "max": 1},
                            },
                        },
                    },
                    "dlio": {
                        "enabled": False,
                        "endpoints": {
                            "odom": {
                                "topic_key": "dlio_odom",
                                "type_key": "odometry",
                                "publishers": {"min": 1, "max": 1},
                            },
                            "path": {
                                "topic_key": "dlio_path",
                                "type_key": "path",
                                "publishers": {"min": 1, "max": 1},
                            },
                        },
                    },
                },
            }
        )

    def _valid_graph(self):
        publishers = {
            "/pose_gt": ["gazebo_truth"],
            "/state/odometry": ["state_odom_relay"],
            "/state/odometry/filtered": ["state_odom_filtered_relay"],
            "/cmd_vel_raw": ["move_base"],
            "/cmd_vel": ["cmd_vel_passthrough"],
            "/cmd_drive": ["twist_to_drive"],
            "/thrusters/1/input": ["cmd_drive_to_thrusters"],
            "/thrusters/0/input": ["cmd_drive_to_thrusters"],
            "/mariner/final_pose": ["nav_bridge"],
            "/mariner/status/nav_ok": ["move_base_goal_manager"],
            "/mariner/status/move_base": ["move_base_status_relay"],
            "/oracle/output": ["human_query_relay"],
            "/oracle/status": ["mission_manager"],
            "/oracle/map/anchors": ["anchor_manager"],
            "/sensors/imu/data": ["imu_plugin"],
            "/sensors/lidar/hori/points": ["lidar_plugin"],
            "/sensors/sonar/scan": ["sonar_plugin"],
            "/sensors/camera/f1/image_raw": ["camera_plugin"],
        }
        subscribers = {
            "/cmd_vel_raw": ["cmd_vel_passthrough"],
            "/cmd_vel": ["twist_to_drive"],
            "/cmd_drive": ["cmd_drive_to_thrusters"],
            "/thrusters/1/input": ["gazebo"],
            "/thrusters/0/input": ["gazebo"],
            "/mariner/final_pose": ["move_base_goal_manager"],
            "/mariner/goal": ["nav_bridge"],
            "/mariner/goal_anchor": ["nav_bridge"],
            "/mariner/status/move_base": ["mission_manager"],
            "/oracle/map/anchors": ["nav_bridge"],
            "/sensors/lidar/hori/points": ["move_base"],
        }
        topic_types = {
            "/pose_gt": "nav_msgs/Odometry",
            "/state/odometry": "nav_msgs/Odometry",
            "/state/odometry/filtered": "nav_msgs/Odometry",
            "/cmd_vel_raw": "geometry_msgs/Twist",
            "/cmd_vel": "geometry_msgs/Twist",
            "/cmd_drive": "heron_msgs/Drive",
            "/thrusters/1/input": "geometry_msgs/Wrench",
            "/thrusters/0/input": "geometry_msgs/Wrench",
            "/mariner/final_pose": "geometry_msgs/PoseStamped",
            "/mariner/goal": "geometry_msgs/PoseStamped",
            "/mariner/goal_anchor": "std_msgs/String",
            "/mariner/status/nav_ok": "std_msgs/String",
            "/mariner/status/move_base": "actionlib_msgs/GoalStatusArray",
            "/oracle/input": "oracle/HumanQueryInput",
            "/oracle/output": "std_msgs/String",
            "/oracle/status": "oracle/MissionStatus",
            "/oracle/map/anchors": "oracle/AnchorPoseArray",
            "/sensors/imu/data": "sensor_msgs/Imu",
            "/sensors/lidar/hori/points": "sensor_msgs/PointCloud2",
            "/sensors/sonar/scan": "sensor_msgs/PointCloud2",
            "/sensors/camera/f1/image_raw": "sensor_msgs/Image",
        }
        return publishers, subscribers, topic_types

    def test_contract_accepts_valid_graph(self):
        publishers, subscribers, topic_types = self._valid_graph()
        errors = topic_contract_guard.validate_topic_contract(
            publishers, subscribers, topic_types, self.contract
        )
        self.assertEqual(errors, [])

    def test_contract_rejects_duplicate_odom_publisher(self):
        publishers, subscribers, topic_types = self._valid_graph()
        publishers["/state/odometry"].append("ekf_localization_node")
        errors = topic_contract_guard.validate_topic_contract(
            publishers, subscribers, topic_types, self.contract
        )
        self.assertTrue(any("/state/odometry has 2 publishers" in e for e in errors))

    def test_contract_rejects_legacy_nav_ok_alias(self):
        publishers, subscribers, topic_types = self._valid_graph()
        publishers["/mariner/nav_ok"] = ["legacy_node"]
        topic_types["/mariner/nav_ok"] = "std_msgs/String"
        errors = topic_contract_guard.validate_topic_contract(
            publishers, subscribers, topic_types, self.contract
        )
        self.assertTrue(any("forbidden topic /mariner/nav_ok" in e for e in errors))

    def test_disabled_module_does_not_require_topics(self):
        publishers, subscribers, topic_types = self._valid_graph()
        errors = topic_contract_guard.validate_topic_contract(
            publishers, subscribers, topic_types, self.contract
        )
        self.assertFalse(any("/state/dlio/path" in e for e in errors))

    def test_contract_allows_cmd_vel_publisher_to_be_idle(self):
        publishers, subscribers, topic_types = self._valid_graph()
        publishers.pop("/cmd_vel")
        errors = topic_contract_guard.validate_topic_contract(
            publishers, subscribers, topic_types, self.contract
        )
        self.assertFalse(any("/cmd_vel has 0 publishers" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
