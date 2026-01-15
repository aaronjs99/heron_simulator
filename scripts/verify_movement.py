#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import time


class MovementVerifier:
    def __init__(self):
        rospy.init_node("movement_verifier", anonymous=True)
        self.start_pose = None
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.start_time = time.time()
        self.found = False

    def callback(self, msg):
        try:
            idx = msg.name.index("heron")
            pose = msg.pose[idx]

            if self.start_pose is None:
                self.start_pose = pose
                print(f"Start Pose: x={pose.position.x:.2f}, y={pose.position.y:.2f}")
                self.t0 = time.time()
            else:
                dt = time.time() - self.t0
                dist = (
                    (pose.position.x - self.start_pose.position.x) ** 2
                    + (pose.position.y - self.start_pose.position.y) ** 2
                ) ** 0.5
                print(
                    f"T+{dt:.1f}s: x={pose.position.x:.2f}, y={pose.position.y:.2f}, dist={dist:.3f}m"
                )

                if dt > 5.0:
                    rospy.signal_shutdown("Done")
                    self.found = True
        except ValueError:
            pass


if __name__ == "__main__":
    v = MovementVerifier()
    rospy.spin()
