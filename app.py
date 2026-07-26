#!/usr/bin/env python3

import threading
import time
from datetime import datetime, timezone

import rclpy
from flask import Flask, jsonify, request
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from speech_service import (
    SpeechBusyError,
    SpeechExecutionError,
    SpeechService,
    SpeechValidationError,
)


app = Flask(__name__)

HOST = "0.0.0.0"
PORT = 8090
MOTION_TOPIC = "/cmd_vel"

MAX_LINEAR_X = 0.20
MAX_ANGULAR_Z = 1.00
MAX_DURATION = 2.00

STREAM_PUBLISH_HZ = 20.0
STREAM_DEFAULT_TIMEOUT_SECONDS = 0.75
STREAM_MIN_TIMEOUT_SECONDS = 0.20
STREAM_MAX_TIMEOUT_SECONDS = 2.00

ros_ready = False
ros_error = None
publisher_node = None
publisher_lock = threading.Lock()

motion_lock = threading.RLock()
motion_state = {
    "streaming": False,
    "linear_x": 0.0,
    "angular_z": 0.0,
    "deadline_monotonic": None,
    "last_command_at": None,
    "last_stop_at": None,
    "watchdog_stop_count": 0,
}

speech_service = SpeechService()


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

    executor = None
    node = None

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

        print(
            f"Robot Bridge ROS2 error: {exc}",
            flush=True,
        )

    finally:
        ros_ready = False

        try:
            if executor is not None:
                executor.shutdown()
        except Exception:
            pass

        try:
            if node is not None:
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
            "error": (
                ros_error
                or "ROS2 publisher is not ready."
            ),
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
            "error": (
                f"ROS2 publish failed: {exc}"
            ),
        }


def clear_streaming_state():
    with motion_lock:
        motion_state["streaming"] = False
        motion_state["linear_x"] = 0.0
        motion_state["angular_z"] = 0.0
        motion_state["deadline_monotonic"] = None
        motion_state["last_stop_at"] = now_iso()


def stop_robot():
    clear_streaming_state()

    result = publish_twist(0.0, 0.0)

    if result.get("ok"):
        time.sleep(0.05)

    return result


def set_streaming_motion(
    linear_x,
    angular_z,
    timeout_seconds,
):
    deadline = (
        time.monotonic()
        + float(timeout_seconds)
    )

    timestamp = now_iso()

    with motion_lock:
        motion_state["streaming"] = True
        motion_state["linear_x"] = float(
            linear_x
        )
        motion_state["angular_z"] = float(
            angular_z
        )
        motion_state[
            "deadline_monotonic"
        ] = deadline
        motion_state["last_command_at"] = (
            timestamp
        )

    return {
        "ok": True,
        "linear_x": float(linear_x),
        "angular_z": float(angular_z),
        "watchdog_timeout_seconds": float(
            timeout_seconds
        ),
        "last_command_at": timestamp,
    }


def streaming_motion_loop():
    interval = 1.0 / STREAM_PUBLISH_HZ

    while True:
        started = time.monotonic()

        should_publish_motion = False
        should_publish_stop = False
        linear_x = 0.0
        angular_z = 0.0

        with motion_lock:
            if motion_state["streaming"]:
                deadline = motion_state[
                    "deadline_monotonic"
                ]

                if (
                    deadline is not None
                    and time.monotonic() >= deadline
                ):
                    motion_state[
                        "streaming"
                    ] = False
                    motion_state[
                        "linear_x"
                    ] = 0.0
                    motion_state[
                        "angular_z"
                    ] = 0.0
                    motion_state[
                        "deadline_monotonic"
                    ] = None
                    motion_state[
                        "last_stop_at"
                    ] = now_iso()
                    motion_state[
                        "watchdog_stop_count"
                    ] += 1

                    should_publish_stop = True

                else:
                    linear_x = motion_state[
                        "linear_x"
                    ]
                    angular_z = motion_state[
                        "angular_z"
                    ]

                    should_publish_motion = True

        if should_publish_motion:
            publish_twist(
                linear_x,
                angular_z,
            )

        elif should_publish_stop:
            publish_twist(0.0, 0.0)

            print(
                "Robot Bridge streaming watchdog "
                "published automatic stop.",
                flush=True,
            )

        elapsed = time.monotonic() - started
        remaining = interval - elapsed

        if remaining > 0.0:
            time.sleep(remaining)


def validate_motion_payload(payload):
    try:
        linear_x = float(
            payload.get("linear_x", 0.0)
        )

        angular_z = float(
            payload.get("angular_z", 0.0)
        )

        duration = float(
            payload.get("duration", 0.25)
        )

        streaming = bool(
            payload.get("streaming", False)
        )

        watchdog_timeout = float(
            payload.get(
                "watchdog_timeout",
                STREAM_DEFAULT_TIMEOUT_SECONDS,
            )
        )

    except (TypeError, ValueError):
        return None, {
            "ok": False,
            "error": (
                "linear_x, angular_z, duration, "
                "and watchdog_timeout must be numeric."
            ),
        }

    if abs(linear_x) > MAX_LINEAR_X:
        return None, {
            "ok": False,
            "error": (
                f"linear_x exceeds safe limit "
                f"of {MAX_LINEAR_X}."
            ),
        }

    if abs(angular_z) > MAX_ANGULAR_Z:
        return None, {
            "ok": False,
            "error": (
                f"angular_z exceeds safe limit "
                f"of {MAX_ANGULAR_Z}."
            ),
        }

    if duration <= 0.0 or duration > MAX_DURATION:
        return None, {
            "ok": False,
            "error": (
                "duration must be greater than 0 "
                f"and no more than {MAX_DURATION} "
                "seconds."
            ),
        }

    if (
        watchdog_timeout
        < STREAM_MIN_TIMEOUT_SECONDS
        or watchdog_timeout
        > STREAM_MAX_TIMEOUT_SECONDS
    ):
        return None, {
            "ok": False,
            "error": (
                "watchdog_timeout must be between "
                f"{STREAM_MIN_TIMEOUT_SECONDS} and "
                f"{STREAM_MAX_TIMEOUT_SECONDS} "
                "seconds."
            ),
        }

    return {
        "linear_x": linear_x,
        "angular_z": angular_z,
        "duration": duration,
        "streaming": streaming,
        "watchdog_timeout": watchdog_timeout,
    }, None


@app.route("/status", methods=["GET"])
def status():
    with motion_lock:
        stream_snapshot = {
            "streaming": bool(
                motion_state["streaming"]
            ),
            "linear_x": float(
                motion_state["linear_x"]
            ),
            "angular_z": float(
                motion_state["angular_z"]
            ),
            "last_command_at": motion_state[
                "last_command_at"
            ],
            "last_stop_at": motion_state[
                "last_stop_at"
            ],
            "watchdog_stop_count": int(
                motion_state[
                    "watchdog_stop_count"
                ]
            ),
        }

    return jsonify(
        {
            "ok": bool(ros_ready),
            "service": (
                "mini_pupper_robot_bridge"
            ),
            "timestamp": now_iso(),
            "robot": "mini_pupper_2",
            "status": (
                "READY"
                if ros_ready
                else "ROS_NOT_READY"
            ),
            "motion_topic": MOTION_TOPIC,
            "controller": (
                "/quadruped_controller_node"
            ),
            "ros_ready": ros_ready,
            "ros_error": ros_error,
            "stream_publish_hz": (
                STREAM_PUBLISH_HZ
            ),
            "stream_default_timeout_seconds": (
                STREAM_DEFAULT_TIMEOUT_SECONDS
            ),
            "motion": stream_snapshot,
            "speech": speech_service.status(),
        }
    )


@app.route("/motion", methods=["POST"])
def motion():
    payload = request.get_json(
        silent=True
    ) or {}

    parsed, error = validate_motion_payload(
        payload
    )

    if error is not None:
        return jsonify(error), 400

    linear_x = parsed["linear_x"]
    angular_z = parsed["angular_z"]
    duration = parsed["duration"]
    streaming = parsed["streaming"]
    watchdog_timeout = parsed[
        "watchdog_timeout"
    ]

    if streaming:
        initial_result = publish_twist(
            linear_x=linear_x,
            angular_z=angular_z,
        )

        if not initial_result.get("ok"):
            stop_robot()

            return jsonify(
                {
                    "ok": False,
                    "action": "motion",
                    "mode": "streaming",
                    "timestamp": now_iso(),
                    "error": initial_result.get(
                        "error",
                        "ROS2 motion publish failed.",
                    ),
                    "motion_result": (
                        initial_result
                    ),
                }
            ), 503

        stream_result = set_streaming_motion(
            linear_x=linear_x,
            angular_z=angular_z,
            timeout_seconds=watchdog_timeout,
        )

        return jsonify(
            {
                "ok": True,
                "action": "motion",
                "mode": "streaming",
                "timestamp": now_iso(),
                "linear_x": linear_x,
                "angular_z": angular_z,
                "watchdog_timeout": (
                    watchdog_timeout
                ),
                "automatic_stop": True,
                "returned_immediately": True,
                "stream_result": stream_result,
            }
        )

    clear_streaming_state()

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
                "mode": "bounded",
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
                "mode": "bounded",
                "timestamp": now_iso(),
                "error": (
                    "Motion executed, but "
                    "automatic stop failed."
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
            "mode": "bounded",
            "timestamp": now_iso(),
            "linear_x": linear_x,
            "angular_z": angular_z,
            "duration": duration,
            "automatic_stop": True,
            "returned_immediately": False,
        }
    )


@app.route("/speak", methods=["POST"])
def speak():
    payload = request.get_json(silent=True)

    if not isinstance(payload, dict):
        return jsonify(
            {
                "ok": False,
                "action": "speak",
                "timestamp": now_iso(),
                "error": (
                    "A JSON object containing text is required."
                ),
            }
        ), 400

    timestamp = now_iso()

    try:
        result = speech_service.speak(
            text=payload.get("text"),
            timestamp=timestamp,
        )
    except SpeechValidationError as exc:
        return jsonify(
            {
                "ok": False,
                "action": "speak",
                "timestamp": timestamp,
                "error": str(exc),
            }
        ), 400
    except SpeechBusyError as exc:
        return jsonify(
            {
                "ok": False,
                "action": "speak",
                "timestamp": timestamp,
                "error": str(exc),
            }
        ), 409
    except SpeechExecutionError as exc:
        return jsonify(
            {
                "ok": False,
                "action": "speak",
                "timestamp": timestamp,
                "error": str(exc),
            }
        ), 503

    return jsonify(
        {
            "ok": True,
            "action": "speak",
            "timestamp": timestamp,
            "message": "Speech played on the Mini Pupper.",
            "speech_result": result,
        }
    )


@app.route("/stop", methods=["POST"])
def stop():
    stop_result = stop_robot()

    status_code = (
        200
        if stop_result.get("ok")
        else 503
    )

    return jsonify(
        {
            "ok": bool(
                stop_result.get("ok")
            ),
            "action": "stop",
            "timestamp": now_iso(),
            "message": (
                "Streaming motion cancelled and "
                "zero velocity published to ROS2."
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

    stream_thread = threading.Thread(
        target=streaming_motion_loop,
        daemon=True,
        name="robot-bridge-streaming-motion",
    )

    ros_thread.start()
    stream_thread.start()

    deadline = time.monotonic() + 5.0

    while (
        not ros_ready
        and time.monotonic() < deadline
    ):
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
