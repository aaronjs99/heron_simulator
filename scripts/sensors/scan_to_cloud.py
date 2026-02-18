#!/usr/bin/env python3
"""
Scan to Cloud Converter
-----------------------
Converts 2D LaserScan messages to 3D PointCloud2 messages.
Used in simulation when simulated LiDAR outputs scan data but the stack expects point clouds.

Subscribes:
- lidar_h/scan (LaserScan)

Publishes:
- /sensors/lidar/hori/points (PointCloud2)
"""

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class ScanToCloud:
    """Converts 2D LaserScan messages to 3D PointCloud2 messages.

    Uses laser_geometry.LaserProjection to project 2D scan points into
    3D space for compatibility with point cloud processing pipelines.

    Attributes:
      laser_projector (LaserProjection): Projection utility instance.
      scan_sub (rospy.Subscriber): Subscriber for LaserScan messages.
      cloud_pub (rospy.Publisher): Publisher for PointCloud2 messages.
    """

    def __init__(self):
        """Initialize the ScanToCloud node with subscribers and publishers."""
        rospy.init_node("scan_to_cloud")

        self.laser_projector = LaserProjection()

        # Subscribe to the LaserScan topic
        # Assuming the topic name based on the mariner config we are reverting from (lidar_h/scan)
        self.scan_sub = rospy.Subscriber("lidar_h/scan", LaserScan, self.scan_callback)

        # Publish to the PointCloud2 topic
        # Assuming the topic name based on the mariner config we are reverting to (/sensors/lidar/hori/points)
        self.cloud_pub = rospy.get_publisher(
            "/sensors/lidar/hori/points", PointCloud2, queue_size=1
        )

        rospy.spin()

    def scan_callback(self, scan_msg):
        """Convert LaserScan to PointCloud2 and publish.

        Args:
          scan_msg (LaserScan): Incoming 2D laser scan message.
        """
        try:
            cloud_msg = self.laser_projector.projectLaser(scan_msg)
            self.cloud_pub.publish(cloud_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to project scan to cloud: {e}")


if __name__ == "__main__":
    try:
        ScanToCloud()
    except rospy.ROSInterruptException:
        pass
