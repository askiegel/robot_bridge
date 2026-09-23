from pathlib import Path
import threading

import transient_tf_lookup
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def helper_source():
    return Path(
        transient_tf_lookup.__file__
    ).read_text(
        encoding="utf-8"
    )


def robot_bridge_node_source():
    source = Path(
        "app.py"
    ).read_text(
        encoding="utf-8"
    )

    return source[
        source.index(
            "class RobotBridgePublisher"
        ):
        source.index(
            "def ros_spin():"
        )
    ]


def test_transient_lookup_has_session_api():
    source = helper_source()

    assert (
        "class TransientTfLookup"
        in source
    )

    assert (
        "def session(self):"
        in source
    )


def test_transient_listener_does_not_create_hidden_spin_thread():
    source = helper_source()

    assert (
        "spin_thread=False"
        in source
    )

    assert (
        "SingleThreadedExecutor"
        in source
    )

    assert (
        "threading.Thread("
        in source
    )


def test_transient_executor_has_explicit_shutdown():
    source = helper_source()

    assert (
        "executor.shutdown("
        in source
    )

    assert (
        "thread.join("
        in source
    )

    assert (
        "node.destroy_node()"
        in source
    )


def test_robot_bridge_has_no_permanent_dynamic_tf_subscription():
    source = robot_bridge_node_source()

    assert '"/tf",' not in source
    assert "'/tf'," not in source

    assert (
        "TransformListener("
        not in source
    )


def test_robot_bridge_has_no_permanent_static_tf_subscription():
    source = robot_bridge_node_source()

    assert '"/tf_static",' not in source
    assert "'/tf_static'," not in source


def test_robot_bridge_has_no_second_odom_subscription():
    source = robot_bridge_node_source()

    assert '"/odom",' not in source
    assert "'/odom'," not in source

    assert "'/odom/local'," in source


def test_mapping_pose_remains_demand_driven():
    source = robot_bridge_node_source()

    assert (
        "self.navigation_tf_lookup = "
        "TransientTfLookup()"
        in source
    )

    assert (
        "MappingPoseProvider("
        in source
    )


class FakeBuffer:
    def __init__(self):
        self.dynamic = []
        self.static = []

    def set_transform(self, transform, authority):
        self.dynamic.append((transform, authority))

    def set_transform_static(self, transform, authority):
        self.static.append((transform, authority))


def make_odometry_session():
    session = object.__new__(
        transient_tf_lookup._TransientOdometryTfSession
    )
    session.buffer = FakeBuffer()
    session._validation_lock = threading.Lock()
    session._validation_error = None

    return session


def make_odometry(parent_frame, child_frame):
    message = Odometry()
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.position.z = 3.75
    message.pose.pose.orientation.x = 0.1
    message.pose.pose.orientation.y = 0.2
    message.pose.pose.orientation.z = 0.3
    message.pose.pose.orientation.w = 0.4

    return message


def assert_preserved_transform(transform, message):
    assert transform.header.stamp == message.header.stamp
    assert transform.header.frame_id == message.header.frame_id
    assert transform.child_frame_id == message.child_frame_id
    assert transform.transform.translation.x == message.pose.pose.position.x
    assert transform.transform.translation.y == message.pose.pose.position.y
    assert transform.transform.translation.z == message.pose.pose.position.z
    assert transform.transform.rotation.x == message.pose.pose.orientation.x
    assert transform.transform.rotation.y == message.pose.pose.orientation.y
    assert transform.transform.rotation.z == message.pose.pose.orientation.z
    assert transform.transform.rotation.w == message.pose.pose.orientation.w


def test_navigation_preflight_session_uses_odom_and_static_not_dynamic_tf():
    source = helper_source()

    session = source[
        source.index("class _TransientOdometryTfSession"):
        source.index("class TransientOdometryTfLookup")
    ]

    assert '"/odom",' in session
    assert '"/odom/local",' in session
    assert '"/tf_static",' in session
    assert '"/tf",' not in session
    assert "TransformListener(" not in session


def test_odom_transform_preserves_message_stamp_pose_and_frames():
    session = make_odometry_session()
    message = make_odometry("odom", "base_footprint")

    session._handle_odom(message)

    assert len(session.buffer.dynamic) == 1
    transform, authority = session.buffer.dynamic[0]
    assert authority == "transient_navigation_preflight"
    assert_preserved_transform(transform, message)


def test_local_odom_transform_preserves_message_stamp_pose_and_frames():
    session = make_odometry_session()
    message = make_odometry("base_footprint", "base_link")

    session._handle_local_odom(message)

    assert len(session.buffer.dynamic) == 1
    transform, _ = session.buffer.dynamic[0]
    assert_preserved_transform(transform, message)


def test_unexpected_odometry_frames_fail_closed_without_transform():
    session = make_odometry_session()
    message = make_odometry("unexpected", "base_link")

    session._handle_odom(message)

    assert session.buffer.dynamic == []
    assert "expected odom -> base_footprint" in (
        session.validation_error()
    )


def test_static_transforms_share_the_navigation_preflight_buffer():
    session = make_odometry_session()
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "lidar_link"
    message = TFMessage(transforms=[transform])

    session._handle_static_tf(message)

    assert session.buffer.static == [
        (transform, "transient_navigation_preflight")
    ]


def test_navigation_preflight_session_closes_owned_resources():
    source = helper_source()

    session = source[
        source.index("class _TransientOdometryTfSession"):
        source.index("class TransientOdometryTfLookup")
    ]

    assert "executor.shutdown(" in session
    assert "thread.join(" in session
    assert "node.destroy_subscription(subscription)" in session
    assert "node.destroy_node()" in session
