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

    def _publish_twist(self, linear_x=0.0, angular_z=0.0):
        if linear_x or angular_z:
            raise RuntimeError("legacy_nonzero_twist_refused")
        msg = Twist()
        self._publisher.publish(msg)  # Explicit unconditional emergency zero.

    def publish_motion(self, linear_x=0.0, angular_z=0.0, duration=0.25):
        raise RuntimeError("use_guarded_bounded_bridge_endpoint")

    def stop(self):
        self.initialize()

        for _ in range(10):
            self._publish_twist(0.0, 0.0)
            time.sleep(0.05)


_motion = RosMotion()


def publish_motion(linear_x=0.0, angular_z=0.0, duration=0.25):
    return _motion.publish_motion(linear_x, angular_z, duration)


def stop():
    return _motion.stop()
