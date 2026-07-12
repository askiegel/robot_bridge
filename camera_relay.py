#!/usr/bin/env python3

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


HOST = "0.0.0.0"
PORT = 8091
IMAGE_TOPIC = "/image_raw"
JPEG_QUALITY = 80
FRAME_TIMEOUT_SECONDS = 3.0


app = Flask(__name__)

lock = threading.Lock()
latest_jpeg = None
latest_timestamp = None
latest_frame_monotonic = None
ros_ready = False


class CameraRelayNode(Node):
    def __init__(self):
        super().__init__("camera_http_relay")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
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
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
            )

            if not ok:
                self.get_logger().warning(
                    "Could not encode camera frame as JPEG."
                )
                return

            timestamp = (
                f"{message.header.stamp.sec}."
                f"{message.header.stamp.nanosec:09d}"
            )

            with lock:
                latest_jpeg = encoded.tobytes()
                latest_timestamp = timestamp
                latest_frame_monotonic = time.monotonic()

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
            "service": "mini_pupper_camera_relay",
            "image_topic": IMAGE_TOPIC,
            "camera_running": camera_is_running(),
            "ros_ready": ros_ready,
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
        has_frame = latest_jpeg is not None

    return jsonify(
        {
            "ok": True,
            "service": "mini_pupper_camera_relay",
            "image_topic": IMAGE_TOPIC,
            "camera_running": camera_is_running(),
            "ros_ready": ros_ready,
            "has_frame": has_frame,
            "latest_timestamp": timestamp,
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
                "error": "No camera frame received yet.",
                "camera_running": False,
            }
        ), 503

    return Response(
        jpeg,
        mimetype="image/jpeg",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
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
