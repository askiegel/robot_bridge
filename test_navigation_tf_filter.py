from pathlib import Path

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from navigation_tf_filter import (
    NAVIGATION_TF_FRAME_PAIRS,
    NAVIGATION_TF_TOPIC,
    filter_navigation_tf,
)


def make_transform(parent, child):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child

    return transform


def test_filter_keeps_only_remote_navigation_frames():
    message = TFMessage()

    message.transforms = [
        make_transform("odom", "base_footprint"),
        make_transform("base_footprint", "base_link"),
        make_transform("base_link", "lf1"),
        make_transform("lf1", "lf2"),
        make_transform("rf1", "rf2"),
    ]

    filtered = filter_navigation_tf(message)

    assert filtered is not None

    actual = [
        (
            transform.header.frame_id,
            transform.child_frame_id,
        )
        for transform in filtered.transforms
    ]

    assert actual == [
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
    ]


def test_filter_accepts_leading_slashes():
    message = TFMessage()

    message.transforms = [
        make_transform(
            "/odom",
            "/base_footprint",
        ),
        make_transform(
            "/base_footprint",
            "/base_link",
        ),
    ]

    filtered = filter_navigation_tf(message)

    assert filtered is not None
    assert len(filtered.transforms) == 2


def test_filter_drops_leg_only_message():
    message = TFMessage()

    message.transforms = [
        make_transform("base_link", "lf1"),
        make_transform("lf1", "lf2"),
        make_transform("lf2", "lf3"),
    ]

    assert filter_navigation_tf(message) is None


def test_filter_contract_is_narrow():
    assert NAVIGATION_TF_TOPIC == (
        "/mayday_navigation_tf"
    )

    assert NAVIGATION_TF_FRAME_PAIRS == frozenset(
        {
            ("odom", "base_footprint"),
            ("base_footprint", "base_link"),
        }
    )


def test_robot_bridge_wires_filtered_tf_publisher():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    for required in (
        "NAVIGATION_TF_TOPIC",
        "filter_navigation_tf",
        "self.navigation_tf_publisher",
        "self.navigation_tf_filter_subscription",
        "self.publish_navigation_tf",
        '"/tf",',
    ):
        assert required in source


def test_filter_has_no_motion_capability():
    source = Path(
        "navigation_tf_filter.py"
    ).read_text(
        encoding="utf-8"
    )

    for forbidden in (
        "Twist",
        "cmd_vel",
        "NavigateToPose",
        "ActionClient",
        "servo",
    ):
        assert forbidden not in source
