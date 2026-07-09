import time
import threading

import rclpy
from geometry_msgs.msg import Twist

from robot_bridge.config import MOTION_TOPIC


class RosMotion:
    def __init__(self):
        self._lock = threading.Lock()
        self._initialized = False
        self._node = None
        self._publisher = None

    def initialize(self):
        with self._lock:
            if self._initialized:
                return

            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = rclpy.create_node("robot_bridge_motion")
            self._publisher = self._node.create_publisher(Twist, MOTION_TOPIC, 10)
            self._initialized = True

    def publish_motion(self, linear_x=0.0, angular_z=0.0, duration=0.25):
        self.initialize()

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        end_time = time.time() + float(duration)

        while time.time() < end_time:
            self._publisher.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.02)
            time.sleep(0.05)

    def stop(self):
        self.initialize()

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(5):
            self._publisher.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.02)
            time.sleep(0.05)


_motion = RosMotion()


def publish_motion(linear_x=0.0, angular_z=0.0, duration=0.25):
    return _motion.publish_motion(linear_x, angular_z, duration)


def stop():
    return _motion.stop()
