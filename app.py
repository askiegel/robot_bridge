#!/usr/bin/env python3

import threading
import time
from datetime import datetime, timezone

import rclpy
from flask import Flask, jsonify, request
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


app = Flask(__name__)

HOST = "0.0.0.0"
PORT = 8090
MOTION_TOPIC = "/cmd_vel"

MAX_LINEAR_X = 0.20
MAX_ANGULAR_Z = 1.00
MAX_DURATION = 2.00

ros_ready = False
ros_error = None
publisher_node = None
publisher_lock = threading.Lock()


def now_iso():
    return datetime.now(timezone.utc).isoformat()


class RobotBridgePublisher(Node):
    def __init__(self):
        super().__init__("robot_bridge_publisher")
        self.publisher = self.create_publisher(
            Twist,
            MOTION_TOPIC,
            10,
        )

        self.get_logger().info(
            f"Robot Bridge publisher ready on {MOTION_TOPIC}"
        )

    def publish_motion(self, linear_x, angular_z):
        message = Twist()

        message.linear.x = float(linear_x)
        message.linear.y = 0.0
        message.linear.z = 0.0

        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = float(angular_z)

        self.publisher.publish(message)


def ros_spin():
    global ros_ready
    global ros_error
    global publisher_node

    try:
        rclpy.init(args=None)

        node = RobotBridgePublisher()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        publisher_node = node
        ros_ready = True
        ros_error = None

        executor.spin()

    except Exception as exc:
        ros_ready = False
        ros_error = str(exc)
        print(f"Robot Bridge ROS2 error: {exc}")

    finally:
        ros_ready = False

        try:
            executor.shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def publish_twist(linear_x, angular_z):
    if not ros_ready or publisher_node is None:
        return {
            "ok": False,
            "error": ros_error or "ROS2 publisher is not ready.",
        }

    try:
        with publisher_lock:
            publisher_node.publish_motion(
                linear_x=linear_x,
                angular_z=angular_z,
            )

        return {
            "ok": True,
            "linear_x": float(linear_x),
            "angular_z": float(angular_z),
        }

    except Exception as exc:
        return {
            "ok": False,
            "error": f"ROS2 publish failed: {exc}",
        }


def stop_robot():
    result = publish_twist(0.0, 0.0)

    if result.get("ok"):
        time.sleep(0.05)

    return result


@app.route("/status", methods=["GET"])
def status():
    return jsonify(
        {
            "ok": bool(ros_ready),
            "service": "mini_pupper_robot_bridge",
            "timestamp": now_iso(),
            "robot": "mini_pupper_2",
            "status": "READY" if ros_ready else "ROS_NOT_READY",
            "motion_topic": MOTION_TOPIC,
            "controller": "/quadruped_controller_node",
            "ros_ready": ros_ready,
            "ros_error": ros_error,
        }
    )


@app.route("/motion", methods=["POST"])
def motion():
    payload = request.get_json(silent=True) or {}

    try:
        linear_x = float(payload.get("linear_x", 0.0))
        angular_z = float(payload.get("angular_z", 0.0))
        duration = float(payload.get("duration", 0.25))
    except (TypeError, ValueError):
        return jsonify(
            {
                "ok": False,
                "error": (
                    "linear_x, angular_z, and duration "
                    "must be numeric."
                ),
            }
        ), 400

    if abs(linear_x) > MAX_LINEAR_X:
        return jsonify(
            {
                "ok": False,
                "error": (
                    f"linear_x exceeds safe limit "
                    f"of {MAX_LINEAR_X}."
                ),
            }
        ), 400

    if abs(angular_z) > MAX_ANGULAR_Z:
        return jsonify(
            {
                "ok": False,
                "error": (
                    f"angular_z exceeds safe limit "
                    f"of {MAX_ANGULAR_Z}."
                ),
            }
        ), 400

    if duration <= 0.0 or duration > MAX_DURATION:
        return jsonify(
            {
                "ok": False,
                "error": (
                    "duration must be greater than 0 and "
                    f"no more than {MAX_DURATION} seconds."
                ),
            }
        ), 400

    motion_result = publish_twist(
        linear_x=linear_x,
        angular_z=angular_z,
    )

    if not motion_result.get("ok"):
        stop_robot()

        return jsonify(
            {
                "ok": False,
                "action": "motion",
                "timestamp": now_iso(),
                "error": motion_result.get(
                    "error",
                    "ROS2 motion publish failed.",
                ),
                "motion_result": motion_result,
            }
        ), 503

    time.sleep(duration)

    stop_result = stop_robot()

    if not stop_result.get("ok"):
        return jsonify(
            {
                "ok": False,
                "action": "motion",
                "timestamp": now_iso(),
                "error": (
                    "Motion executed, but automatic stop failed."
                ),
                "linear_x": linear_x,
                "angular_z": angular_z,
                "duration": duration,
                "motion_result": motion_result,
                "stop_result": stop_result,
            }
        ), 500

    return jsonify(
        {
            "ok": True,
            "action": "motion",
            "timestamp": now_iso(),
            "linear_x": linear_x,
            "angular_z": angular_z,
            "duration": duration,
            "automatic_stop": True,
        }
    )


@app.route("/stop", methods=["POST"])
def stop():
    stop_result = stop_robot()
    status_code = 200 if stop_result.get("ok") else 503

    return jsonify(
        {
            "ok": bool(stop_result.get("ok")),
            "action": "stop",
            "timestamp": now_iso(),
            "message": (
                "Zero-velocity command published to ROS2."
                if stop_result.get("ok")
                else "ROS2 stop publish failed."
            ),
            "stop_result": stop_result,
        }
    ), status_code


def main():
    ros_thread = threading.Thread(
        target=ros_spin,
        daemon=True,
        name="robot-bridge-ros",
    )
    ros_thread.start()

    deadline = time.monotonic() + 5.0

    while not ros_ready and time.monotonic() < deadline:
        if ros_error:
            break
        time.sleep(0.05)

    app.run(
        host=HOST,
        port=PORT,
        debug=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
