#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class ScanToCloud:
    def __init__(self):
        rospy.init_node('scan_to_cloud')
        
        self.laser_projector = LaserProjection()
        
        # Subscribe to the LaserScan topic
        # Assuming the topic name based on the mariner config we are reverting from (lidar_h/scan)
        self.scan_sub = rospy.Subscriber("lidar_h/scan", LaserScan, self.scan_callback)
        
        # Publish to the PointCloud2 topic
        # Assuming the topic name based on the mariner config we are reverting to (lidar_h/velodyne_points)
        self.cloud_pub = rospy.Publisher("lidar_h/velodyne_points", PointCloud2, queue_size=1)
        
        rospy.spin()

    def scan_callback(self, scan_msg):
        # Convert the scan to a point cloud
        # We don't filter or anything, just pure projection
        try:
            cloud_msg = self.laser_projector.projectLaser(scan_msg)
            self.cloud_pub.publish(cloud_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to project scan to cloud: {e}")

if __name__ == '__main__':
    try:
        ScanToCloud()
    except rospy.ROSInterruptException:
        pass
