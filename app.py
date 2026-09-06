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
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_msgs.msg import TFMessage

from transient_tf_lookup import TransientTfLookup

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
from mapping_pose import MappingPoseProvider
from mapping_control import (
    MappingConflictError,
    MappingControl,
    MappingControlError,
)
from navigation_goal_service import (
    NavigationGoalCancelledError,
    NavigationGoalConflictError,
    NavigationGoalError,
    NavigationGoalService,
    NavigationGoalTimeoutError,
    NavigationGoalUnavailableError,
    NavigationGoalValidationError,
)
from navigation_control import (
    NavigationConflictError,
    NavigationControl,
    NavigationControlError,
)
from mapping_navigation_control import (
    MappingNavigationConflictError,
    MappingNavigationControl,
    MappingNavigationControlError,
)
from planning_control import (
    PlanningConflictError,
    PlanningControl,
    PlanningControlError,
)
from planning_path_service import (
    PlanningPathConflictError,
    PlanningPathError,
    PlanningPathService,
    PlanningPathTimeoutError,
    PlanningPathUnavailableError,
    PlanningPathValidationError,
)
from planning_localization_initializer import (
    PlanningLocalizationConflictError,
    PlanningLocalizationError,
    PlanningLocalizationInitializer,
    PlanningLocalizationTimeoutError,
    PlanningLocalizationUnavailableError,
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

MAX_LINEAR_X = 0.50
MAX_ANGULAR_Z = 1.00
MAX_DURATION = 2.00

STREAM_PUBLISH_HZ = 20.0
STREAM_DEFAULT_TIMEOUT_SECONDS = 0.75
STREAM_MIN_TIMEOUT_SECONDS = 0.20
STREAM_MAX_TIMEOUT_SECONDS = 2.00

NAVIGATION_PREFLIGHT_MAX_SENSOR_AGE_SECONDS = 1.00
NAVIGATION_PREFLIGHT_TF_TIMEOUT_SECONDS = 1.00

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
planning_control = PlanningControl()
navigation_control = NavigationControl()
mapping_navigation_control = MappingNavigationControl(
    mapping_state_provider=mapping_control.snapshot,
    action_server_ready_provider=lambda: bool(
        publisher_node is not None
        and publisher_node
        .navigation_goal_service
        .action_server_ready()
    ),
)
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
        'planning': planning_control.snapshot(),
        'navigation': navigation_control.snapshot(),
        'mapping_navigation': (
            mapping_navigation_control.snapshot()
        ),
    },
)

atexit.register(localization_control.shutdown)
atexit.register(mapping_control.shutdown)
atexit.register(planning_control.shutdown)
atexit.register(navigation_control.shutdown)
atexit.register(mapping_navigation_control.shutdown)


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

        self.navigation_preflight_lock = threading.Lock()
        self.latest_scan_stamp = None
        self.latest_scan_frame = None
        self.latest_scan_received_at = None
        self.latest_local_odom_received_at = None

        # Mapping pose may still require map -> odom,
        # so it uses a short-lived TF listener only when
        # explicitly requested.
        self.navigation_tf_lookup = TransientTfLookup()

        # Navigation TF is intentionally demand-driven.
        #
        # Normal Robot Bridge operation must not consume /tf,
        # /tf_static, or a second /odom stream. Exact scan-time
        # validation creates a short-lived full-TF listener only
        # while navigation startup preflight is executing.

        self.mapping_pose_provider = MappingPoseProvider(
            self,
            self.navigation_tf_lookup,
        )

        self.planning_path_service = (
            PlanningPathService(self)
        )
        self.navigation_goal_service = (
            NavigationGoalService(self)
        )
        self.planning_localization_initializer = (
            PlanningLocalizationInitializer(
                self,
                pose_clearer=(
                    localization_telemetry.clear
                ),
            )
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.update_lidar,
            qos_profile_sensor_data,
        )

        self.local_odom_subscription = self.create_subscription(
            Odometry,
            '/odom/local',
            self.update_local_odom,
            10,
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

    def update_lidar(self, message):
        lidar_telemetry.update(message)

        with self.navigation_preflight_lock:
            self.latest_scan_stamp = message.header.stamp
            self.latest_scan_frame = (
                message.header.frame_id.lstrip('/')
            )
            self.latest_scan_received_at = time.monotonic()

    def update_local_odom(self, message):
        with self.navigation_preflight_lock:
            self.latest_local_odom_received_at = (
                time.monotonic()
            )

    def navigation_start_preflight(self):
        checked_at = time.monotonic()

        with self.navigation_preflight_lock:
            odom_received_at = (
                self.latest_local_odom_received_at
            )

        failures = []

        if odom_received_at is None:
            failures.append(
                'No /odom/local message has been received.'
            )
            odom_age = None

        else:
            odom_age = (
                checked_at
                - odom_received_at
            )

            if (
                odom_age
                > NAVIGATION_PREFLIGHT_MAX_SENSOR_AGE_SECONDS
            ):
                failures.append(
                    '/odom/local is stale '
                    f'({odom_age:.3f} seconds old).'
                )

        scan_stamp = None
        scan_frame = None
        scan_received_at = None
        scan_age = None

        if not failures:
            # A newly received LaserScan may carry a timestamp
            # substantially older than its local receive time.
            #
            # Therefore "received after listener startup" alone
            # is insufficient. Keep advancing through fresh
            # scans until the temporary TF buffer can resolve
            # one at its exact ROS timestamp.
            with self.navigation_tf_lookup.session() as tf_lookup:
                listener_started_at = time.monotonic()

                capture_deadline = (
                    listener_started_at
                    + max(
                        3.0,
                        (
                            2.0
                            * NAVIGATION_PREFLIGHT_MAX_SENSOR_AGE_SECONDS
                        ),
                    )
                )

                last_attempted_scan_received_at = None
                last_tf_error = None
                fresh_scan_seen = False

                while (
                    time.monotonic()
                    < capture_deadline
                ):
                    with self.navigation_preflight_lock:
                        candidate_stamp = (
                            self.latest_scan_stamp
                        )
                        candidate_frame = (
                            self.latest_scan_frame
                        )
                        candidate_received_at = (
                            self.latest_scan_received_at
                        )

                    candidate_is_new = (
                        candidate_stamp is not None
                        and bool(candidate_frame)
                        and candidate_received_at is not None
                        and candidate_received_at
                        > listener_started_at
                        and (
                            last_attempted_scan_received_at
                            is None
                            or candidate_received_at
                            > last_attempted_scan_received_at
                        )
                    )

                    if not candidate_is_new:
                        time.sleep(0.01)
                        continue

                    fresh_scan_seen = True

                    last_attempted_scan_received_at = (
                        candidate_received_at
                    )

                    candidate_age = (
                        time.monotonic()
                        - candidate_received_at
                    )

                    if (
                        candidate_age
                        > NAVIGATION_PREFLIGHT_MAX_SENSOR_AGE_SECONDS
                    ):
                        time.sleep(0.01)
                        continue

                    try:
                        transform = (
                            tf_lookup.lookup_transform(
                                'odom',
                                candidate_frame,
                                Time.from_msg(
                                    candidate_stamp
                                ),
                                timeout=Duration(
                                    seconds=(
                                        NAVIGATION_PREFLIGHT_TF_TIMEOUT_SECONDS
                                    ),
                                ),
                            )
                        )

                    except TransformException as exc:
                        # Most early candidates will fail with
                        # past extrapolation while the transient
                        # buffer is still warming. That scan can
                        # never become valid, so advance to the
                        # next LaserScan instead of retrying it.
                        last_tf_error = exc
                        time.sleep(0.01)
                        continue

                    # Exact-time lookup succeeded. This is the
                    # scan whose timestamp navigation startup
                    # has actually validated.
                    scan_stamp = candidate_stamp
                    scan_frame = candidate_frame
                    scan_received_at = (
                        candidate_received_at
                    )

                    scan_age = (
                        time.monotonic()
                        - scan_received_at
                    )

                    # Keep an explicit reference until success
                    # is established; lookup_transform() itself
                    # is the validity check.
                    _ = transform

                    break

                if scan_stamp is None:
                    if not fresh_scan_seen:
                        failures.append(
                            'No fresh /scan message was received '
                            'during transient TF capture.'
                        )

                    else:
                        detail = ''

                        if last_tf_error is not None:
                            detail = (
                                ' Last TF error: '
                                + str(last_tf_error)
                            )

                        failures.append(
                            'No fresh /scan message had an '
                            'exact-time transform '
                            'odom -> scan_frame within the '
                            'transient TF capture window.'
                            + detail
                        )

        return {
            'ok': not failures,
            'odom_topic': '/odom/local',
            'scan_topic': '/scan',
            'target_frame': 'odom',
            'scan_frame': scan_frame,
            'odom_age_seconds': odom_age,
            'scan_age_seconds': scan_age,
            'failures': failures,
        }

    def initialize_planning_localization(self):
        return (
            self.planning_localization_initializer
            .initialize()
        )

    def refresh_planning_localization_pose(self):
        return (
            self.planning_localization_initializer
            .refresh_pose()
        )

    def compute_path(self, payload):
        return self.planning_path_service.compute(
            payload
        )

    def mapping_pose_snapshot(self):
        return self.mapping_pose_provider.snapshot()

    def execute_mapping_navigation_goal(self, payload):
        pose_snapshot = self.mapping_pose_snapshot()

        return self.navigation_goal_service.execute(
            payload,
            pose_snapshot,
        )

    def execute_navigation_goal(self, payload):
        try:
            self.refresh_planning_localization_pose()
        except PlanningLocalizationError as exc:
            raise NavigationGoalUnavailableError(
                'Stationary localization pose refresh '
                f'failed: {exc}'
            ) from exc

        pose_snapshot = None

        for attempt in range(60):
            pose_snapshot = localization_telemetry.snapshot()

            if pose_snapshot.get('available') is True:
                break

            if attempt < 59:
                time.sleep(0.05)

        return self.navigation_goal_service.execute(
            payload,
            pose_snapshot,
        )

    def cancel_navigation_goal(self):
        return (
            self.navigation_goal_service
            .cancel_active()
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
        executor = MultiThreadedExecutor(
            num_threads=2,
        )
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


def cancel_navigation_goal():
    if publisher_node is None:
        return {
            'active': False,
            'cancel_requested': False,
            'cancel_signal_sent': False,
        }

    try:
        return publisher_node.cancel_navigation_goal()
    except Exception as exc:
        return {
            'active': True,
            'cancel_requested': True,
            'cancel_signal_sent': False,
            'error': str(exc),
        }


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


@app.route(
    "/telemetry/mapping-pose",
    methods=["GET"],
)
def live_mapping_pose_status():
    mapping = mapping_control.snapshot()

    runtime_active = bool(
        mapping.get("running")
        and mapping.get("owned")
    )

    if not runtime_active:
        telemetry = {
            "available": False,
            "status": "MAPPING_STOPPED",
            "source": "cartographer_tf",
            "age_seconds": None,
            "error": None,
            "pose": None,
        }
    elif not ros_ready or publisher_node is None:
        telemetry = {
            "available": False,
            "status": "ROS_NOT_READY",
            "source": "cartographer_tf",
            "age_seconds": None,
            "error": (
                ros_error
                or "ROS2 publisher is not ready."
            ),
            "pose": None,
        }
    else:
        try:
            telemetry = (
                publisher_node.mapping_pose_snapshot()
            )
        except Exception as exc:
            telemetry = {
                "available": False,
                "status": "TF_UNAVAILABLE",
                "source": "cartographer_tf",
                "age_seconds": None,
                "error": str(exc),
                "pose": None,
            }

    available = bool(
        runtime_active
        and telemetry.get("available")
    )

    response = jsonify({
        "ok": available,
        "service": "mini_pupper_robot_bridge",
        "runtime_active": runtime_active,
        "mapping": mapping,
        "telemetry": telemetry,
        "timestamp": now_iso(),
        "source": "live_cartographer_tf",
        "read_only": True,
    })

    response.headers[
        "Access-Control-Allow-Origin"
    ] = "*"

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
    """Return whether an owned or discovered AMCL runtime is active."""
    if not ros_ready or publisher_node is None:
        return False

    navigation = navigation_control.snapshot()

    if (
        navigation.get('running')
        and navigation.get('owned')
    ):
        return True

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

    mapping_navigation = (
        mapping_navigation_control.snapshot()
    )

    if (
        mapping_navigation.get("running")
        or mapping_navigation.get("owned")
        or mapping_navigation.get("pid") is not None
    ):
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': (
                'Mapping navigation is active or still '
                'owned; mapping start was refused. Stop '
                'mapping navigation first.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
            'mapping_navigation': mapping_navigation,
        }), 409

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

    if planning_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': (
                'Planning is running; mapping was '
                'not started.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
        }), 409

    if navigation_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'mapping_start',
            'timestamp': timestamp,
            'error': (
                'Navigation is running; mapping was '
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

    mapping_navigation = (
        mapping_navigation_control.snapshot()
    )

    if (
        mapping_navigation.get("running")
        or mapping_navigation.get("owned")
        or mapping_navigation.get("pid") is not None
    ):
        return jsonify({
            'ok': False,
            'action': 'mapping_save_candidate',
            'timestamp': timestamp,
            'error': (
                'Mapping navigation is active or still '
                'owned; candidate export was refused. Stop '
                'mapping navigation first.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
            'mapping_navigation': mapping_navigation,
        }), 409

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

    mapping_navigation = (
        mapping_navigation_control.snapshot()
    )

    if (
        mapping_navigation.get("running")
        or mapping_navigation.get("owned")
        or mapping_navigation.get("pid") is not None
    ):
        return jsonify({
            'ok': False,
            'action': 'mapping_stop',
            'timestamp': timestamp,
            'error': (
                'Mapping navigation is active or still '
                'owned; mapping stop was refused. Stop '
                'mapping navigation first.'
            ),
            'stop_result': stop_result,
            'mapping': mapping_control.snapshot(),
            'mapping_navigation': mapping_navigation,
        }), 409

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


\
@app.route(
    "/mapping-navigation/status",
    methods=["GET"],
)
def mapping_navigation_control_status():
    return jsonify({
        "ok": True,
        "service": "mini_pupper_robot_bridge",
        "timestamp": now_iso(),
        "mapping_navigation": (
            mapping_navigation_control.snapshot()
        ),
    })


@app.route(
    "/mapping-navigation/start",
    methods=["POST"],
)
def mapping_navigation_start():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get("ok"):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Safety zero could not be published; "
                "mapping navigation was not started."
            ),
            "stop_result": stop_result,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    mapping = mapping_control.snapshot()

    if not (
        mapping.get("running")
        and mapping.get("owned")
    ):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Owned live Cartographer mapping "
                "runtime is required."
            ),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 409

    conflicts = (
        (
            "Saved-map navigation",
            navigation_control.snapshot(),
        ),
        (
            "Planning",
            planning_control.snapshot(),
        ),
        (
            "Standalone localization",
            localization_control.snapshot(),
        ),
    )

    for name, state in conflicts:
        if state.get("running"):
            return jsonify({
                "ok": False,
                "action": "mapping_navigation_start",
                "timestamp": timestamp,
                "error": (
                    f"{name} is running; mapping "
                    "navigation was not started."
                ),
                "stop_result": stop_result,
                "mapping": mapping,
                "mapping_navigation": (
                    mapping_navigation_control.snapshot()
                ),
            }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                ros_error
                or "ROS2 is not ready; mapping "
                "navigation was not started."
            ),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    try:
        preflight = (
            publisher_node.navigation_start_preflight()
        )
    except Exception as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Mapping-navigation startup "
                f"preflight failed: {exc}"
            ),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    if not preflight.get("ok"):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Mapping-navigation startup "
                "preflight rejected the request."
            ),
            "preflight": preflight,
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    try:
        mapping_pose = (
            publisher_node.mapping_pose_snapshot()
        )

        NavigationGoalService.validate_pose(
            mapping_pose
        )
    except NavigationGoalValidationError as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Live mapping pose is not ready: "
                f"{exc}"
            ),
            "preflight": preflight,
            "mapping_pose": (
                mapping_pose
                if "mapping_pose" in locals()
                else None
            ),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503
    except Exception as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": (
                "Live mapping pose check failed: "
                f"{exc}"
            ),
            "preflight": preflight,
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    try:
        result = mapping_navigation_control.start(
            timestamp
        )
    except MappingNavigationConflictError as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": str(exc),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 409
    except MappingNavigationControlError as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_start",
            "timestamp": timestamp,
            "error": str(exc),
            "stop_result": stop_result,
            "mapping": mapping,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    status_code = (
        201
        if result.get("started")
        else 200
    )

    return jsonify({
        "ok": True,
        "action": "mapping_navigation_start",
        "timestamp": timestamp,
        "message": (
            "Guarded live-mapping navigation "
            "runtime started without submitting "
            "a goal."
            if result.get("started")
            else (
                "Guarded live-mapping navigation "
                "runtime is already running."
            )
        ),
        "stop_result": stop_result,
        "mapping": mapping,
        "mapping_pose": mapping_pose,
        "mapping_navigation": result,
    }), status_code


@app.route(
    "/mapping-navigation/goal",
    methods=["POST"],
)
def mapping_navigation_goal():
    initial_stop_result = stop_robot()
    timestamp = now_iso()

    mapping_navigation = (
        mapping_navigation_control.snapshot()
    )

    if not initial_stop_result.get("ok"):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": (
                "Safety zero could not be published; "
                "the mapping-navigation goal was "
                "not submitted."
            ),
            "initial_stop_result": (
                initial_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation
            ),
        }), 503

    if (
        not mapping_navigation.get("running")
        or not mapping_navigation.get("owned")
        or not mapping_navigation.get(
            "goal_submission_enabled"
        )
    ):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": (
                "Owned live-mapping navigation "
                "runtime is not ready for goals."
            ),
            "initial_stop_result": (
                initial_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation
            ),
        }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": (
                ros_error
                or "ROS2 navigation client is "
                "not ready."
            ),
            "initial_stop_result": (
                initial_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation
            ),
        }), 503

    payload = request.get_json(silent=True)

    try:
        result = (
            publisher_node
            .execute_mapping_navigation_goal(
                payload
            )
        )
    except NavigationGoalValidationError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 400
    except NavigationGoalConflictError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 409
    except NavigationGoalCancelledError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 409
    except NavigationGoalTimeoutError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 504
    except NavigationGoalUnavailableError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503
    except NavigationGoalError as exc:
        final_stop_result = stop_robot()

        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": str(exc),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    final_stop_result = stop_robot()

    if not final_stop_result.get("ok"):
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_goal",
            "timestamp": timestamp,
            "error": (
                "Mapping-navigation goal completed "
                "but the final safety zero could "
                "not be published."
            ),
            "initial_stop_result": (
                initial_stop_result
            ),
            "final_stop_result": (
                final_stop_result
            ),
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
            "result": result,
        }), 503

    return jsonify({
        "ok": True,
        "action": "mapping_navigation_goal",
        "timestamp": timestamp,
        "message": (
            "One bounded live-mapping navigation "
            "goal completed."
        ),
        "initial_stop_result": (
            initial_stop_result
        ),
        "final_stop_result": (
            final_stop_result
        ),
        "mapping_navigation": (
            mapping_navigation_control.snapshot()
        ),
        "result": result,
    }), 200


@app.route(
    "/mapping-navigation/stop",
    methods=["POST"],
)
def mapping_navigation_stop():
    cancel_result = cancel_navigation_goal()
    stop_result = stop_robot()
    timestamp = now_iso()

    try:
        result = mapping_navigation_control.stop(
            timestamp
        )
    except MappingNavigationControlError as exc:
        return jsonify({
            "ok": False,
            "action": "mapping_navigation_stop",
            "timestamp": timestamp,
            "error": str(exc),
            "cancel_result": cancel_result,
            "stop_result": stop_result,
            "mapping_navigation": (
                mapping_navigation_control.snapshot()
            ),
        }), 503

    return jsonify({
        "ok": bool(stop_result.get("ok")),
        "action": "mapping_navigation_stop",
        "timestamp": timestamp,
        "message": (
            "Guarded live-mapping navigation "
            "runtime stopped."
            if result.get("stopped")
            else (
                "Guarded live-mapping navigation "
                "runtime was already stopped."
            )
        ),
        "cancel_result": cancel_result,
        "stop_result": stop_result,
        "mapping_navigation": result,
    }), (
        200
        if stop_result.get("ok")
        else 503
    )


@app.route("/navigation/status", methods=["GET"])
def navigation_control_status():
    return jsonify({
        'ok': True,
        'service': 'mini_pupper_robot_bridge',
        'timestamp': now_iso(),
        'navigation': navigation_control.snapshot(),
    })


@app.route("/navigation/start", methods=["POST"])
def navigation_start():
    stop_result = stop_robot()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': now_iso(),
            'error': (
                'Safety zero could not be published; '
                'navigation was not started.'
            ),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    timestamp = now_iso()

    conflicts = (
        (
            'Mapping',
            mapping_control.snapshot(),
        ),
        (
            'Planning',
            planning_control.snapshot(),
        ),
        (
            'Standalone localization',
            localization_control.snapshot(),
        ),
    )

    for name, state in conflicts:
        if state.get('running'):
            return jsonify({
                'ok': False,
                'action': 'navigation_start',
                'timestamp': timestamp,
                'error': (
                    f'{name} is running; navigation '
                    'was not started.'
                ),
                'stop_result': stop_result,
                'navigation': (
                    navigation_control.snapshot()
                ),
            }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 is not ready; navigation '
                'was not started.'
            ),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    try:
        preflight = (
            publisher_node.navigation_start_preflight()
        )
    except Exception as exc:
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': timestamp,
            'error': (
                'Navigation startup preflight failed: '
                f'{exc}'
            ),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    if not preflight.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': timestamp,
            'error': (
                'Navigation startup preflight rejected '
                'the request.'
            ),
            'preflight': preflight,
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    localization_telemetry.clear()

    try:
        result = navigation_control.start(timestamp)
    except NavigationConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 409
    except NavigationControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'navigation_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    status_code = (
        201
        if result.get('started')
        else 200
    )

    return jsonify({
        'ok': True,
        'action': 'navigation_start',
        'timestamp': timestamp,
        'message': (
            'Guarded navigation runtime started '
            'without submitting a goal.'
            if result.get('started')
            else (
                'Guarded navigation runtime is '
                'already running.'
            )
        ),
        'stop_result': stop_result,
        'navigation': result,
    }), status_code



@app.route(
    "/navigation/initialize-localization",
    methods=["POST"],
)
def navigation_initialize_localization():
    initial_stop_result = stop_robot()
    timestamp = now_iso()
    navigation = navigation_control.snapshot()

    if not initial_stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'localization was not initialized.'
            ),
            'initial_stop_result': (
                initial_stop_result
            ),
            'navigation': navigation,
        }), 503

    if (
        not navigation.get('running')
        or not navigation.get('owned')
    ):
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Owned guarded navigation runtime '
                'is not active.'
            ),
            'initial_stop_result': (
                initial_stop_result
            ),
            'navigation': navigation,
        }), 409

    if not ros_ready or publisher_node is None:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 publisher is not ready.'
            ),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': navigation,
        }), 503

    payload = request.get_json(silent=True)

    if payload not in (None, {}):
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Navigation localization '
                'initialization does not accept '
                'request parameters.'
            ),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': navigation,
        }), 400

    localization_telemetry.clear()

    try:
        result = (
            publisher_node
            .initialize_planning_localization()
        )
    except PlanningLocalizationConflictError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': (
                navigation_control.snapshot()
            ),
        }), 409
    except PlanningLocalizationUnavailableError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': (
                navigation_control.snapshot()
            ),
        }), 503
    except PlanningLocalizationTimeoutError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': (
                navigation_control.snapshot()
            ),
        }), 504
    except PlanningLocalizationError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': (
                navigation_control.snapshot()
            ),
        }), 422

    final_stop_result = stop_robot()

    if not final_stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': (
                'navigation_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Localization initialization '
                'completed but the final safety '
                'zero could not be published.'
            ),
            'initial_stop_result': (
                initial_stop_result
            ),
            'final_stop_result': final_stop_result,
            'navigation': (
                navigation_control.snapshot()
            ),
            'initialization': result,
        }), 503

    return jsonify({
        'ok': True,
        'action': (
            'navigation_initialize_localization'
        ),
        'timestamp': timestamp,
        'message': (
            'AMCL global localization and '
            'stationary scan updates were '
            'requested for guarded navigation.'
        ),
        'initial_stop_result': (
            initial_stop_result
        ),
        'final_stop_result': final_stop_result,
        'navigation': (
            navigation_control.snapshot()
        ),
        'initialization': result,
    }), 200


@app.route("/navigation/goal", methods=["POST"])
def navigation_goal():
    initial_stop_result = stop_robot()
    timestamp = now_iso()
    navigation = navigation_control.snapshot()

    if not initial_stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'the navigation goal was not submitted.'
            ),
            'initial_stop_result': initial_stop_result,
            'navigation': navigation,
        }), 503

    if (
        not navigation.get('running')
        or not navigation.get('owned')
    ):
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': (
                'Owned guarded navigation runtime '
                'is not active.'
            ),
            'initial_stop_result': initial_stop_result,
            'navigation': navigation,
        }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 navigation client is not ready.'
            ),
            'initial_stop_result': initial_stop_result,
            'navigation': navigation,
        }), 503

    payload = request.get_json(silent=True)

    try:
        result = (
            publisher_node
            .execute_navigation_goal(payload)
        )
    except NavigationGoalValidationError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 400
    except NavigationGoalConflictError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 409
    except NavigationGoalCancelledError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 409
    except NavigationGoalTimeoutError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 504
    except NavigationGoalUnavailableError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503
    except NavigationGoalError as exc:
        final_stop_result = stop_robot()
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': str(exc),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    final_stop_result = stop_robot()

    if not final_stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'navigation_goal',
            'timestamp': timestamp,
            'error': (
                'Navigation completed but the final '
                'safety zero could not be published.'
            ),
            'initial_stop_result': initial_stop_result,
            'final_stop_result': final_stop_result,
            'navigation': navigation_control.snapshot(),
            'result': result,
        }), 503

    return jsonify({
        'ok': True,
        'action': 'navigation_goal',
        'timestamp': timestamp,
        'message': (
            'One bounded guarded navigation goal '
            'completed.'
        ),
        'initial_stop_result': initial_stop_result,
        'final_stop_result': final_stop_result,
        'navigation': navigation_control.snapshot(),
        'result': result,
    }), 200


@app.route("/navigation/stop", methods=["POST"])
def navigation_stop():
    cancel_result = cancel_navigation_goal()
    stop_result = stop_robot()
    timestamp = now_iso()

    try:
        result = navigation_control.stop(timestamp)
    except NavigationControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'navigation_stop',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'navigation': navigation_control.snapshot(),
        }), 503

    localization_telemetry.clear()

    return jsonify({
        'ok': bool(stop_result.get('ok')),
        'action': 'navigation_stop',
        'timestamp': timestamp,
        'message': (
            'Guarded navigation runtime stopped.'
            if result.get('stopped')
            else (
                'Guarded navigation runtime was '
                'already stopped.'
            )
        ),
        'cancel_result': cancel_result,
        'stop_result': stop_result,
        'navigation': result,
    }), (
        200
        if stop_result.get('ok')
        else 503
    )


@app.route("/planning/status", methods=["GET"])
def planning_control_status():
    return jsonify({
        'ok': True,
        'service': 'mini_pupper_robot_bridge',
        'timestamp': now_iso(),
        'planning': planning_control.snapshot(),
    })


@app.route("/planning/start", methods=["POST"])
def planning_start():
    stop_result = stop_robot()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': now_iso(),
            'error': (
                'Safety zero could not be published; '
                'planning was not started.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    timestamp = now_iso()

    if mapping_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': timestamp,
            'error': (
                'Mapping is running; planning was '
                'not started.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 409

    if localization_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': timestamp,
            'error': (
                'Standalone localization is running; '
                'planning was not started.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 409

    if navigation_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': timestamp,
            'error': (
                'Navigation is running; planning was '
                'not started.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 409

    localization_telemetry.clear()

    try:
        result = planning_control.start(timestamp)
    except PlanningConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 409
    except PlanningControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_start',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    status_code = (
        201
        if result.get('started')
        else 200
    )

    return jsonify({
        'ok': True,
        'action': 'planning_start',
        'timestamp': timestamp,
        'message': (
            'Planning-only Nav2 started.'
            if result.get('started')
            else 'Planning-only Nav2 is already running.'
        ),
        'stop_result': stop_result,
        'planning': result,
    }), status_code


@app.route(
    "/planning/initialize-localization",
    methods=["POST"],
)
def planning_initialize_localization():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'localization was not initialized.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    planning = planning_control.snapshot()

    if (
        not planning.get('running')
        or not planning.get('owned')
    ):
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Owned planning runtime is not active.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 publisher is not ready.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 503

    payload = request.get_json(silent=True)

    if payload not in (None, {}):
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': (
                'Planning localization initialization '
                'does not accept request parameters.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 400

    localization_telemetry.clear()

    try:
        result = (
            publisher_node
            .initialize_planning_localization()
        )
    except PlanningLocalizationConflictError as exc:
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 409
    except PlanningLocalizationUnavailableError as exc:
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 503
    except PlanningLocalizationTimeoutError as exc:
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 504
    except PlanningLocalizationError as exc:
        return jsonify({
            'ok': False,
            'action': (
                'planning_initialize_localization'
            ),
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 422

    return jsonify({
        'ok': True,
        'action': (
            'planning_initialize_localization'
        ),
        'timestamp': timestamp,
        'message': (
            'AMCL global localization and stationary '
            'scan updates were requested.'
        ),
        'stop_result': stop_result,
        'planning': planning,
        'initialization': result,
    }), 200


@app.route(
    "/planning/refresh-localization",
    methods=["POST"],
)
def planning_refresh_localization():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'localization was not refreshed.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    planning = planning_control.snapshot()

    if (
        not planning.get('running')
        or not planning.get('owned')
    ):
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': (
                'Owned planning runtime is not active.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 publisher is not ready.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 503

    payload = request.get_json(silent=True)

    if payload not in (None, {}):
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': (
                'Planning localization refresh does not '
                'accept request parameters.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 400

    try:
        result = (
            publisher_node
            .refresh_planning_localization_pose()
        )
    except PlanningLocalizationConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 409
    except PlanningLocalizationUnavailableError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 503
    except PlanningLocalizationTimeoutError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 504
    except PlanningLocalizationError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_refresh_localization',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 503

    return jsonify({
        'ok': True,
        'action': 'planning_refresh_localization',
        'timestamp': timestamp,
        'message': (
            'One stationary AMCL pose refresh was requested.'
        ),
        'refresh': result,
        'stop_result': stop_result,
        'planning': planning,
    }), 200


@app.route(
    "/planning/compute-path",
    methods=["POST"],
)
def planning_compute_path():
    stop_result = stop_robot()
    timestamp = now_iso()

    if not stop_result.get('ok'):
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': (
                'Safety zero could not be published; '
                'no path was requested.'
            ),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    planning = planning_control.snapshot()

    if (
        not planning.get('running')
        or not planning.get('owned')
    ):
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': (
                'Owned planning runtime is not '
                'active.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 409

    if not ros_ready or publisher_node is None:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': (
                ros_error
                or 'ROS2 publisher is not ready.'
            ),
            'stop_result': stop_result,
            'planning': planning,
        }), 503

    payload = request.get_json(silent=True)

    try:
        result = publisher_node.compute_path(
            payload
        )
    except PlanningPathValidationError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 400
    except PlanningPathConflictError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 409
    except PlanningPathUnavailableError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 503
    except PlanningPathTimeoutError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 504
    except PlanningPathError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_compute_path',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning,
        }), 422

    return jsonify({
        'ok': True,
        'action': 'planning_compute_path',
        'timestamp': timestamp,
        'message': (
            'A read-only path was computed. '
            'The path was not executed.'
        ),
        'stop_result': stop_result,
        'planning': planning,
        'path': result,
    }), 200


@app.route("/planning/stop", methods=["POST"])
def planning_stop():
    stop_result = stop_robot()
    timestamp = now_iso()

    try:
        result = planning_control.stop(timestamp)
    except PlanningControlError as exc:
        return jsonify({
            'ok': False,
            'action': 'planning_stop',
            'timestamp': timestamp,
            'error': str(exc),
            'stop_result': stop_result,
            'planning': planning_control.snapshot(),
        }), 503

    localization_telemetry.clear()

    return jsonify({
        'ok': bool(stop_result.get('ok')),
        'action': 'planning_stop',
        'timestamp': timestamp,
        'message': (
            'Planning-only Nav2 stopped.'
            if result.get('stopped')
            else 'Planning-only Nav2 was already stopped.'
        ),
        'stop_result': stop_result,
        'planning': result,
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

    timestamp = now_iso()

    if planning_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'localization_start',
            'timestamp': timestamp,
            'error': (
                'Planning is running; standalone '
                'localization was not started.'
            ),
            'stop_result': stop_result,
            'localization': (
                localization_control.snapshot()
            ),
        }), 409

    if navigation_control.snapshot().get('running'):
        return jsonify({
            'ok': False,
            'action': 'localization_start',
            'timestamp': timestamp,
            'error': (
                'Navigation is running; standalone '
                'localization was not started.'
            ),
            'stop_result': stop_result,
            'localization': (
                localization_control.snapshot()
            ),
        }), 409

    localization_telemetry.clear()

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
    cancel_result = cancel_navigation_goal()
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
            "cancel_result": cancel_result,
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
