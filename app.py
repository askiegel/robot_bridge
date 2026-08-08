#!/usr/bin/env python3

import atexit
import threading
import time
from datetime import datetime, timezone

import rclpy
from flask import Flask, jsonify, request
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from cartographer_ros_msgs.msg import SubmapList
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from candidate_map_telemetry import (
    CandidateMapTelemetry,
)
from lidar_telemetry import LidarTelemetry
from live_mapping_telemetry import LiveMappingTelemetry
from mapping_readiness_telemetry import (
    MappingReadinessTelemetry,
)
from localization_control import (
    LocalizationConflictError,
    LocalizationControl,
    LocalizationControlError,
)
from localization_telemetry import LocalizationTelemetry
from mapping_control import (
    MappingConflictError,
    MappingControl,
    MappingControlError,
)
from map_telemetry import SavedMapTelemetry
from map_promotion import (
    MapPromotion,
    MapPromotionConflictError,
    MapPromotionError,
)

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
lidar_telemetry = LidarTelemetry()
live_mapping_telemetry = LiveMappingTelemetry()
localization_telemetry = LocalizationTelemetry()
localization_control = LocalizationControl()
mapping_control = MappingControl()
mapping_requirements = mapping_control.snapshot()
mapping_readiness_telemetry = (
    MappingReadinessTelemetry(
        minimum_submaps=(
            mapping_requirements[
                'candidate_minimum_submaps'
            ]
        ),
        minimum_mature_submaps=(
            mapping_requirements[
                'candidate_minimum_mature_submaps'
            ]
        ),
        minimum_mature_version=(
            mapping_requirements[
                'candidate_minimum_mature_version'
            ]
        ),
    )
)
map_telemetry = SavedMapTelemetry()
candidate_map_telemetry = CandidateMapTelemetry(
    validated_map_telemetry=map_telemetry,
)
map_promotion = MapPromotion(
    candidate_map_telemetry=candidate_map_telemetry,
    validated_map_telemetry=map_telemetry,
    runtime_state_provider=lambda: {
        'mapping': mapping_control.snapshot(),
        'localization': localization_control.snapshot(),
    },
)

atexit.register(localization_control.shutdown)
atexit.register(mapping_control.shutdown)


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

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            lidar_telemetry.update,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Robot Bridge publisher ready on {MOTION_TOPIC}"
        )
        self.localization_subscription = (
            self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                localization_telemetry.update,
                10,
            )
        )

        live_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.live_mapping_subscription = (
            self.create_subscription(
                OccupancyGrid,
                '/map',
                live_mapping_telemetry.update,
                live_map_qos,
            )
        )

        self.mapping_readiness_subscription = (
            self.create_subscription(
                SubmapList,
                '/submap_list',
                mapping_readiness_telemetry.update,
                10,
            )
        )

        self.get_logger().info(
            "Robot Bridge LiDAR telemetry ready on /scan"
        )
        self.get_logger().info(
            "Robot Bridge localization telemetry ready on /amcl_pose"
        )
        self.get_logger().info(
            "Robot Bridge live mapping telemetry ready on /map"
        )
        self.get_logger().info(
            "Robot Bridge mapping readiness ready on "
            "/submap_list"
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


@app.route("/telemetry/lidar", methods=["GET"])
def lidar_status():
    telemetry = lidar_telemetry.snapshot()
    available = telemetry["available"]

    response = jsonify(
        {
            "ok": available,
            "service": "mini_pupper_robot_bridge",
            "timestamp": now_iso(),
            "topic": "/scan",
            "telemetry": telemetry,
        }
    )
    response.headers["Access-Control-Allow-Origin"] = "*"

    return response, 200 if available else 503


@app.route(
    "/telemetry/mapping-map",
    methods=["GET"],
)
def live_mapping_status():
    mapping = mapping_control.snapshot()
    runtime_active = bool(
        mapping.get('running')
        and mapping.get('owned')
    )

    if not runtime_active:
        live_mapping_telemetry.clear()

    telemetry = live_mapping_telemetry.snapshot()

    if not runtime_active:
        telemetry['status'] = 'MAPPING_STOPPED'

    available = (
        runtime_active
        and telemetry['available']
    )

    response = jsonify({
        'ok': available,
        'service': 'mini_pupper_robot_bridge',
        'runtime_active': runtime_active,
        'mapping': mapping,
        'telemetry': telemetry,
        'timestamp': now_iso(),
        'topic': '/map',
        'source': 'live_cartographer_map',
        'read_only': True,
        'authoritative': False,
    })
    response.headers['Access-Control-Allow-Origin'] = '*'

    return response, 200 if available else 503


@app.route("/telemetry/map", methods=["GET"])
def map_status():
    telemetry = map_telemetry.snapshot()
    available = telemetry['available']

    response = jsonify({
        'ok': available,
        'service': 'mini_pupper_robot_bridge',
        'telemetry': telemetry,
        'timestamp': now_iso(),
        'source': 'validated_saved_map',
    })
    response.headers['Access-Control-Allow-Origin'] = '*'

    return response, 200 if available else 503


def localization_runtime_active():
    """Return whether the conservative AMCL node is currently present."""
    if not ros_ready or publisher_node is None:
        return False

    try:
        with publisher_lock:
            node_names = publisher_node.get_node_names()

        return 'amcl' in {
            str(name).strip('/')
            for name in node_names
        }

    except Exception:
        return False


@app.route(
    "/telemetry/map-candidates",
    methods=["GET"],
)
def candidate_map_status():
    telemetry = candidate_map_telemetry.snapshot()
    available = telemetry['available']

    response = jsonify({
        'ok': available,
        'service': 'mini_pupper_robot_bridge',
        'telemetry': telemetry,
        'timestamp': now_iso(),
        'source': 'candidate_map_review',
    })
    response.headers['Access-Control-Allow-Origin'] = '*'

    return response, 200 if available else 503


@app.route(
    "/map/promote-candidate",
    methods=["POST"],
)
def promote_candidate_map():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'candidate promotion was not attempted.'
            ),
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 503

    payload = request.get_json(
        silent=True
    )

    if not isinstance(payload, dict):
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': 'A JSON request body is required.',
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 400

    allowed_keys = {
        'candidate_name',
        'confirmation',
    }

    if set(payload) != allowed_keys:
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': (
                'Exactly candidate_name and confirmation '
                'must be supplied.'
            ),
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 400

    if (
        mapping_control.snapshot().get('running')
        or localization_control.snapshot().get('running')
    ):
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': (
                'Mapping or localization is active; '
                'candidate promotion is blocked.'
            ),
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 409

    try:
        result = map_promotion.promote(
            candidate_name=payload.get(
                'candidate_name'
            ),
            confirmation=payload.get(
                'confirmation'
            ),
            timestamp=timestamp,
        )
    except MapPromotionConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 409
    except MapPromotionError as exc:
        return jsonify({
            'ok': False,
            'action': 'map_promote_candidate',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'promotion': map_promotion.snapshot(),
        }), 400

    return jsonify({
        'ok': True,
        'action': 'map_promote_candidate',
        'timestamp': timestamp,
        'message': (
            'Reviewed candidate was promoted with a '
            'validated-map backup.'
        ),
        'stop_result': stop_result,
        'promotion': result,
    }), 201


@app.route("/telemetry/localization", methods=["GET"])
def localization_status():
    runtime_active = localization_runtime_active()

    if not runtime_active:
        localization_telemetry.clear()

    telemetry = localization_telemetry.snapshot()

    if not runtime_active:
        telemetry['status'] = 'LOCALIZATION_STOPPED'

    available = (
        runtime_active
        and telemetry['available']
    )

    response = jsonify({
        'ok': available,
        'service': 'mini_pupper_robot_bridge',
        'runtime_active': runtime_active,
        'telemetry': telemetry,
        'timestamp': now_iso(),
        'topic': '/amcl_pose',
    })
    response.headers['Access-Control-Allow-Origin'] = '*'

    return response, 200 if available else 503


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


def mapping_snapshot_with_readiness():
    """Return mapping ownership and live readiness."""
    mapping = mapping_control.snapshot()
    runtime_active = bool(
        mapping.get('running')
        and mapping.get('owned')
    )

    if not runtime_active:
        mapping_readiness_telemetry.clear()

    mapping['readiness'] = (
        mapping_readiness_telemetry.snapshot(
            runtime_active=runtime_active,
        )
    )
    return mapping


@app.route("/mapping/status", methods=["GET"])
def mapping_control_status():
    return jsonify({
        'ok': True,
        'service': 'mini_pupper_robot_bridge',
        'timestamp': now_iso(),
        'mapping': mapping_snapshot_with_readiness(),
    })


@app.route("/mapping/start", methods=["POST"])
def mapping_start():
    stop_result = stop_robot()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': now_iso(),
            'error': (
                'Safety zero could not be published; '
                'mapping was not started.'
            ),
            'stop_result': stop_result,
        }), 503

    timestamp = now_iso()

    if localization_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': (
                'Localization is running; mapping was '
                'not started.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 409

    if not mapping_control.snapshot().get('running'):
        live_mapping_telemetry.clear()
        mapping_readiness_telemetry.clear()

    try:
        result = mapping_control.start(timestamp)
    except MappingConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 409
    except MappingControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 503

    status_code = (
        201
        if result.get('started')
        else 200
    )

    return jsonify({
        'ok': True,
        'action': 'mapping_start',
        'timestamp': timestamp,
        'message': (
            'Headless mapping started.'
            if result.get('started')
            else 'Headless mapping is already running.'
        ),
        'stop_result': stop_result,
        'mapping': result,
    }), status_code


@app.route(
    "/mapping/save-candidate",
    methods=["POST"],
)
def mapping_save_candidate():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'mapping_save_candidate',
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'candidate export was not attempted.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 503

    try:
        result = mapping_control.save_candidate(
            timestamp,
        )
    except MappingControlError as exc:
        if not mapping_control.snapshot().get('running'):
            live_mapping_telemetry.clear()
            mapping_readiness_telemetry.clear()

        return jsonify({
            'ok': False,
            'action': 'mapping_save_candidate',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 409

    live_mapping_telemetry.clear()
    mapping_readiness_telemetry.clear()

    return jsonify({
        'ok': True,
        'action': 'mapping_save_candidate',
        'timestamp': timestamp,
        'message': (
            'Mapping stopped and candidate map saved '
            'for explicit review.'
        ),
        'stop_result': stop_result,
        'mapping': result,
        'candidate': result['candidate'],
    }), 201


@app.route("/mapping/stop", methods=["POST"])
def mapping_stop():
    stop_result = stop_robot()
    timestamp = now_iso()

    try:
        result = mapping_control.stop(timestamp)
    except MappingControlError as exc:
        if not mapping_control.snapshot().get('running'):
            live_mapping_telemetry.clear()
            mapping_readiness_telemetry.clear()

        return jsonify({
            'ok': False,
            'action': 'mapping_stop',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 503

    live_mapping_telemetry.clear()
    mapping_readiness_telemetry.clear()

    return jsonify({
        'ok': bool(stop_result.get('ok')),
        'action': 'mapping_stop',
        'timestamp': timestamp,
        'message': (
            'Headless mapping stopped without saving.'
            if result.get('stopped')
            else 'Headless mapping was already stopped.'
        ),
        'stop_result': stop_result,
        'mapping': result,
    }), (
        200
        if stop_result.get('ok')
        else 503
    )


@app.route("/localization/status", methods=["GET"])
def localization_control_status():
    return jsonify({
        'ok': True,
        'service': 'mini_pupper_robot_bridge',
        'timestamp': now_iso(),
        'localization': localization_control.snapshot(),
    })


@app.route("/localization/start", methods=["POST"])
def localization_start():
    stop_result = stop_robot()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'localization_start',
            'timestamp': now_iso(),
            'error': (
                'Safety zero could not be published; '
                'localization was not started.'
            ),
            'stop_result': stop_result,
        }), 503

    localization_telemetry.clear()
    timestamp = now_iso()

    try:
        result = localization_control.start(
            timestamp,
        )
    except LocalizationConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'localization_start',
            'timestamp': timestamp,
            'error': str(exc),
            'localization': (
                localization_control.snapshot()
            ),
        }), 409
    except LocalizationControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'localization_start',
            'timestamp': timestamp,
            'error': str(exc),
            'localization': (
                localization_control.snapshot()
            ),
        }), 503

    status_code = (
        201
        if result.get('started')
        else 200
    )

    return jsonify({
        'ok': True,
        'action': 'localization_start',
        'timestamp': timestamp,
        'message': (
            'Conservative localization started.'
            if result.get('started')
            else 'Conservative localization is already running.'
        ),
        'stop_result': stop_result,
        'localization': result,
    }), status_code


@app.route("/localization/stop", methods=["POST"])
def localization_stop():
    stop_result = stop_robot()
    timestamp = now_iso()

    try:
        result = localization_control.stop(
            timestamp,
        )
    except LocalizationControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'localization_stop',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'localization': (
                localization_control.snapshot()
            ),
        }), 503

    localization_telemetry.clear()

    return jsonify({
        'ok': bool(stop_result.get('ok')),
        'action': 'localization_stop',
        'timestamp': timestamp,
        'message': (
            'Conservative localization stopped.'
            if result.get('stopped')
            else 'Conservative localization was already stopped.'
        ),
        'stop_result': stop_result,
        'localization': result,
    }), (
        200
        if stop_result.get('ok')
        else 503
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
