#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


class MappingPoseProvider:
    """
    Read Mayday's latest Cartographer map-frame pose from TF.

    Cartographer owns map -> odom.
    Robot state estimation owns odom -> base_link.

    This provider performs no localization initialization and
    publishes no transforms or motion commands.
    """

    TARGET_FRAME = "map"
    SOURCE_FRAME = "base_link"
    TF_TIMEOUT_SECONDS = 0.25

    def __init__(self, node, tf_buffer):
        self._node = node
        self._tf_buffer = tf_buffer

    def snapshot(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.TARGET_FRAME,
                self.SOURCE_FRAME,
                Time(),
                timeout=Duration(
                    seconds=self.TF_TIMEOUT_SECONDS
                ),
            )
        except TransformException as exc:
            return {
                "available": False,
                "status": "TF_UNAVAILABLE",
                "source": "cartographer_tf",
                "age_seconds": None,
                "error": str(exc),
                "pose": None,
            }

        stamp = transform.header.stamp

        stamp_nanoseconds = (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )

        now_nanoseconds = int(
            self._node.get_clock().now().nanoseconds
        )

        age_seconds = max(
            0.0,
            (
                now_nanoseconds
                - stamp_nanoseconds
            )
            / 1_000_000_000.0,
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        return {
            "available": True,
            "status": "READY",
            "source": "cartographer_tf",
            "age_seconds": float(age_seconds),
            "error": None,
            "pose": {
                "frame_id": self.TARGET_FRAME,
                "source_frame_id": self.SOURCE_FRAME,
                "stamp_seconds": (
                    stamp_nanoseconds
                    / 1_000_000_000.0
                ),
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            },
        }
