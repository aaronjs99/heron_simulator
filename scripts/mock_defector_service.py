#!/usr/bin/env python3
"""
Mock Defector Service for Simulation Testing.

This node simulates the behavior of the Defector inspection system:
- Provides a 'capture_and_analyze' service that returns random defect results.
- Publishes a synthetic debug image stream to '/defector/ai_debug_view' to
  visualize the "inspection" process in simulation.

Author: Heron Dev Team
"""
import math
import random
import time
from typing import Dict, Any

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse


class MockDefectorService:
    """
    Simulates the Defector inspection service and debug stream.
    """

    def __init__(self):
        """Initialize the mock service, publisher, and timer."""
        # ROS Service
        self.service = rospy.Service(
            "/defector/capture_and_analyze", Trigger, self.handle_capture
        )
        
        # ROS Publisher
        self.image_pub = rospy.Publisher("/defector/ai_debug_view", Image, queue_size=1)
        
        # Utilities
        self.bridge = CvBridge()
        self.capture_count = 0
        
        # Start visualization timer (10Hz for smoother animation)
        rospy.Timer(rospy.Duration(0.1), self.publish_debug_image)
        
        rospy.loginfo("Mock defector service ready at /defector/capture_and_analyze")
        rospy.loginfo("Publishing synthetic view to /defector/ai_debug_view")

    def handle_capture(self, req: Trigger) -> TriggerResponse:
        """
        Handle a capture request by generating a random result.
        
        Args:
            req: Trigger request (empty).
            
        Returns:
            TriggerResponse with success=True and a descriptive message.
        """
        self.capture_count += 1
        
        # Simulate random inspection results
        # 30% chance of detecting a defect
        has_defect = random.random() < 0.3
        
        if has_defect:
            coverage = round(random.uniform(0.01, 0.15), 4)
            bbox = self._generate_random_bbox()
            message = (
                f"[SIM] Capture #{self.capture_count}: DEFECT DETECTED - "
                f"coverage={coverage:.2%}, bbox=({bbox['x']},{bbox['y']},{bbox['w']},{bbox['h']})"
            )
        else:
            message = f"[SIM] Capture #{self.capture_count}: No defects detected, surface appears normal."
        
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

    def publish_debug_image(self, event):
        """
        Publish a synthetic debug image to simulate the AI view.
        
        Generates a dynamic image with text, status, and moving elements.
        """
        # Create a dark background
        img = np.zeros((480, 640, 3), np.uint8)
        
        # Add Header
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, 'HERON SIMULATION', (20, 50), font, 1.2, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Add Status Info
        cv2.putText(img, f'System Status: ONLINE', (20, 100), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(img, f'Capture Count: {self.capture_count}', (20, 130), font, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, f'Time: {rospy.Time.now().to_sec():.1f}', (20, 160), font, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
        
        # Animated "Scanner" Bar
        t = time.time()
        scan_y = int(240 + 200 * math.sin(t * 2.0))
        cv2.line(img, (0, scan_y), (640, scan_y), (0, 0, 255), 2)
        cv2.putText(img, 'SCANNING...', (500, scan_y - 10), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        # Rotating Indicator
        center_x, center_y = 550, 80
        radius = 30
        angle = t * 3.0
        end_x = int(center_x + radius * math.cos(angle))
        end_y = int(center_y + radius * math.sin(angle))
        cv2.circle(img, (center_x, center_y), radius, (100, 100, 100), 2)
        cv2.line(img, (center_x, center_y), (end_x, end_y), (0, 255, 0), 2)

        try:
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_pub.publish(msg)
        except Exception:
            # Squelch errors during shutdown or encoding
            pass

    def _generate_random_bbox(self) -> Dict[str, int]:
        """Generate a random bounding box dictionary."""
        return {
            "x": random.randint(50, 400),
            "y": random.randint(50, 300),
            "w": random.randint(20, 100),
            "h": random.randint(20, 80),
        }


def main():
    """Main entry point."""
    rospy.init_node("mock_defector_service")
    MockDefectorService()
    rospy.loginfo("Mock defector service node started (simulation mode)")
    rospy.spin()


if __name__ == "__main__":
    main()
