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


class LiveMappingTelemetry:
    """Store the latest transient Cartographer occupancy grid."""

    def __init__(self, monotonic_clock=None, utc_clock=None):
        self._lock = threading.Lock()
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )
        self._map = None
        self._received_monotonic = None
        self._received_at = None
        self._error = None

    @staticmethod
    def _yaw_from_quaternion(orientation):
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)

        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def update(self, message):
        """Record one immutable JSON-compatible map snapshot."""
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        cells = [int(value) for value in message.data]
        expected = width * height
        error = None

        if width <= 0 or height <= 0:
            error = 'Map dimensions must be positive.'
        elif resolution <= 0.0:
            error = 'Map resolution must be positive.'
        elif len(cells) != expected:
            error = (
                'Occupancy data length does not match '
                'map dimensions.'
            )
        elif any(
            value < -1 or value > 100
            for value in cells
        ):
            error = (
                'Occupancy data contains a value outside '
                'the ROS range of -1 through 100.'
            )

        if error is not None:
            with self._lock:
                self._error = error
            return

        stamp = message.header.stamp
        stamp_seconds = (
            float(stamp.sec)
            + float(stamp.nanosec) / 1_000_000_000.0
        )
        origin = message.info.origin

        occupancy_map = {
            'frame_id': str(message.header.frame_id),
            'name': 'live_cartographer_map',
            'stamp_seconds': stamp_seconds,
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin': {
                'x': float(origin.position.x),
                'y': float(origin.position.y),
                'yaw': self._yaw_from_quaternion(
                    origin.orientation
                ),
            },
            'cell_count': expected,
            'unknown_cell_count': cells.count(-1),
            'free_cell_count': cells.count(0),
            'occupied_cell_count': cells.count(100),
            'probability_cell_count': sum(
                1
                for value in cells
                if 0 < value < 100
            ),
            'encoding': 'ros_occupancy_probabilities',
            'unknown_value': -1,
            'free_value': 0,
            'occupied_value': 100,
            'cells': cells,
            'source': {
                'topic': '/map',
                'runtime': 'cartographer',
                'mutable': True,
                'authoritative': False,
            },
        }

        received_at = self._utc_clock().isoformat()
        received_monotonic = self._monotonic_clock()

        with self._lock:
            self._map = occupancy_map
            self._received_at = received_at
            self._received_monotonic = received_monotonic
            self._error = None

    def clear(self):
        """Discard every grid from a previous mapping session."""
        with self._lock:
            self._map = None
            self._received_monotonic = None
            self._received_at = None
            self._error = None

    def snapshot(self):
        """Return the latest grid without exposing mutable state."""
        now = self._monotonic_clock()

        with self._lock:
            if self._map is None:
                return {
                    'available': False,
                    'status': (
                        'INVALID_MAP'
                        if self._error
                        else 'WAITING_FOR_MAP'
                    ),
                    'received_at': None,
                    'age_seconds': None,
                    'error': self._error,
                    'map': None,
                }

            occupancy_map = dict(self._map)
            occupancy_map['origin'] = dict(
                self._map['origin']
            )
            occupancy_map['source'] = dict(
                self._map['source']
            )
            occupancy_map['cells'] = list(
                self._map['cells']
            )

            return {
                'available': True,
                'status': 'READY',
                'received_at': self._received_at,
                'age_seconds': max(
                    0.0,
                    now - self._received_monotonic,
                ),
                'error': None,
                'map': occupancy_map,
            }
