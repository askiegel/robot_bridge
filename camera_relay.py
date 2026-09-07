#!/usr/bin/env python3

import threading
import time

import rclpy
from flask import Flask, Response, jsonify
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


HOST = "0.0.0.0"
PORT = 8091
IMAGE_TOPIC = "/image_raw/compressed"
FRAME_TIMEOUT_SECONDS = 3.0


app = Flask(__name__)

lock = threading.Lock()

latest_jpeg = None
latest_timestamp = None
latest_frame_monotonic = None

ros_ready = False


class CameraRelayNode(Node):

    def __init__(self):
        super().__init__(
            "camera_http_relay"
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Camera HTTP relay subscribed to {IMAGE_TOPIC}"
        )

    def image_callback(self, message):
        global latest_jpeg
        global latest_timestamp
        global latest_frame_monotonic

        try:
            jpeg = bytes(
                message.data
            )

            if not jpeg:
                self.get_logger().warning(
                    "Received empty compressed camera frame."
                )
                return

            timestamp = (
                f"{message.header.stamp.sec}."
                f"{message.header.stamp.nanosec:09d}"
            )

            now = time.monotonic()

            with lock:
                latest_jpeg = jpeg
                latest_timestamp = timestamp
                latest_frame_monotonic = now

        except Exception as exc:
            self.get_logger().error(
                f"Camera frame processing failed: {exc}"
            )


def ros_spin():
    global ros_ready

    rclpy.init(args=None)

    node = CameraRelayNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_ready = True

    try:
        executor.spin()

    finally:
        ros_ready = False

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def camera_is_running():
    with lock:
        last_frame = latest_frame_monotonic

    if last_frame is None:
        return False

    return (
        time.monotonic() - last_frame
        <= FRAME_TIMEOUT_SECONDS
    )


@app.get("/")
def root():
    return jsonify(
        {
            "ok": True,
            "service":
                "mini_pupper_camera_relay",
            "image_topic":
                IMAGE_TOPIC,
            "camera_running":
                camera_is_running(),
            "ros_ready":
                ros_ready,
            "endpoints": [
                "/status",
                "/camera/latest.jpg",
            ],
        }
    )


@app.get("/status")
def status():
    with lock:
        timestamp = latest_timestamp
        has_frame = (
            latest_jpeg is not None
        )

    return jsonify(
        {
            "ok": True,
            "service":
                "mini_pupper_camera_relay",
            "image_topic":
                IMAGE_TOPIC,
            "camera_running":
                camera_is_running(),
            "ros_ready":
                ros_ready,
            "has_frame":
                has_frame,
            "latest_timestamp":
                timestamp,
        }
    )


@app.get("/camera/latest.jpg")
def latest_camera_frame():
    with lock:
        jpeg = latest_jpeg

    if jpeg is None:
        return jsonify(
            {
                "ok": False,
                "error":
                    "No camera frame received yet.",
                "camera_running":
                    False,
            }
        ), 503

    return Response(
        jpeg,
        mimetype="image/jpeg",
        headers={
            "Cache-Control":
                "no-store, no-cache, "
                "must-revalidate",
        },
    )


def main():
    ros_thread = threading.Thread(
        target=ros_spin,
        daemon=True,
        name="camera-relay-ros",
    )

    ros_thread.start()

    app.run(
        host=HOST,
        port=PORT,
        debug=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
