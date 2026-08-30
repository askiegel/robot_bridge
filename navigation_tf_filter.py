"""Filter Mayday dynamic TF for remote mapping/navigation."""

from tf2_msgs.msg import TFMessage


NAVIGATION_TF_TOPIC = "/mayday_navigation_tf"

NAVIGATION_TF_FRAME_PAIRS = frozenset(
    {
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
    }
)


def frame_pair(transform):
    """Return a normalized parent/child frame pair."""

    return (
        transform.header.frame_id.lstrip("/"),
        transform.child_frame_id.lstrip("/"),
    )


def filter_navigation_tf(message):
    """
    Keep only Mayday dynamic transforms needed remotely.

    Native /tf remains untouched. Articulated leg transforms are
    deliberately excluded from /mayday_navigation_tf.
    """

    selected = [
        transform
        for transform in message.transforms
        if frame_pair(transform) in NAVIGATION_TF_FRAME_PAIRS
    ]

    if not selected:
        return None

    outgoing = TFMessage()
    outgoing.transforms = selected

    return outgoing
