#!/usr/bin/env python3
import os
import unittest


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SENSOR_XACRO = os.path.join(REPO_ROOT, "urdf", "ig_handle_sensors.urdf.xacro")


class SonarUrdfProfileTests(unittest.TestCase):
    def test_sonar_is_configured_as_point_cloud_sensor(self):
        with open(SENSOR_XACRO, "r", encoding="utf-8") as handle:
            content = handle.read()

        self.assertIn('<plugin name="sonar_plugin" filename="libgazebo_ros_velodyne_laser.so">', content)
        self.assertIn('<topicName>/sensors/sonar/scan</topicName>', content)
        self.assertIn('<samples>64</samples>', content)
        self.assertIn('<samples>12</samples>', content)


if __name__ == "__main__":
    unittest.main()
