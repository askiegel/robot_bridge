#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import threading
import time
from datetime import datetime
from datetime import timezone


class LocalizationTelemetry:
    """Store the latest AMCL pose as transient read-only telemetry."""

    def __init__(self, monotonic_clock=None, utc_clock=None):
        self._lock = threading.Lock()
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )
        self._pose = None
        self._received_monotonic = None
        self._received_at = None

    @staticmethod
    def _finite(value, name):
        value = float(value)

        if not math.isfinite(value):
            raise ValueError(
                f'Localization {name} must be finite.'
            )

        return value

    @classmethod
    def _yaw_from_quaternion(cls, quaternion):
        x = cls._finite(quaternion.x, 'quaternion x')
        y = cls._finite(quaternion.y, 'quaternion y')
        z = cls._finite(quaternion.z, 'quaternion z')
        w = cls._finite(quaternion.w, 'quaternion w')

        norm = math.sqrt(
            x * x
            + y * y
            + z * z
            + w * w
        )

        if norm <= 1e-12:
            raise ValueError(
                'Localization quaternion has zero magnitude.'
            )

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        return (
            math.atan2(sin_yaw, cos_yaw),
            {
                'x': x,
                'y': y,
                'z': z,
                'w': w,
            },
        )

    def update(self, message):
        """Record one immutable JSON-compatible AMCL pose."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        covariance = [
            self._finite(value, 'covariance')
            for value in message.pose.covariance
        ]

        if len(covariance) != 36:
            raise ValueError(
                'Localization covariance must contain 36 values.'
            )

        yaw_radians, quaternion = self._yaw_from_quaternion(
            orientation
        )

        stamp = message.header.stamp
        stamp_seconds = (
            float(stamp.sec)
            + float(stamp.nanosec) / 1_000_000_000.0
        )

        x = self._finite(position.x, 'position x')
        y = self._finite(position.y, 'position y')
        z = self._finite(position.z, 'position z')

        pose = {
            'frame_id': str(message.header.frame_id),
            'stamp_seconds': stamp_seconds,
            'position': {
                'x': x,
                'y': y,
                'z': z,
            },
            'orientation': quaternion,
            'yaw_radians': yaw_radians,
            'yaw_degrees': math.degrees(yaw_radians),
            'covariance': covariance,
            'uncertainty': {
                'x_variance': covariance[0],
                'y_variance': covariance[7],
                'yaw_variance': covariance[35],
                'x_standard_deviation': math.sqrt(
                    max(0.0, covariance[0])
                ),
                'y_standard_deviation': math.sqrt(
                    max(0.0, covariance[7])
                ),
                'yaw_standard_deviation_radians': math.sqrt(
                    max(0.0, covariance[35])
                ),
            },
        }

        received_at = self._utc_clock().isoformat()
        received_monotonic = self._monotonic_clock()

        with self._lock:
            self._pose = pose
            self._received_at = received_at
            self._received_monotonic = received_monotonic

    def clear(self):
        """Discard every pose from a previous localization session."""
        with self._lock:
            self._pose = None
            self._received_monotonic = None
            self._received_at = None

    def snapshot(self):
        """Return the latest pose without exposing mutable state."""
        now = self._monotonic_clock()

        with self._lock:
            if self._pose is None:
                return {
                    'available': False,
                    'status': 'WAITING_FOR_LOCALIZATION',
                    'received_at': None,
                    'age_seconds': None,
                    'pose': None,
                }

            age = max(
                0.0,
                now - self._received_monotonic,
            )

            pose = dict(self._pose)
            pose['position'] = dict(
                self._pose['position']
            )
            pose['orientation'] = dict(
                self._pose['orientation']
            )
            pose['uncertainty'] = dict(
                self._pose['uncertainty']
            )
            pose['covariance'] = list(
                self._pose['covariance']
            )

            return {
                'available': True,
                'status': 'READY',
                'received_at': self._received_at,
                'age_seconds': age,
                'pose': pose,
            }
