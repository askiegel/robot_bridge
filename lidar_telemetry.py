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


class LidarTelemetry:
    """Store the latest LaserScan as transient read-only telemetry."""

    def __init__(self, monotonic_clock=None, utc_clock=None):
        self._lock = threading.Lock()
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )
        self._scan = None
        self._received_monotonic = None
        self._received_at = None

    @staticmethod
    def _finite_or_none(value):
        value = float(value)

        if math.isfinite(value):
            return value

        return None

    def update(self, message):
        """Record one immutable JSON-compatible scan snapshot."""
        ranges = [
            self._finite_or_none(value)
            for value in message.ranges
        ]

        valid_ranges = [
            value
            for value in ranges
            if value is not None
        ]

        stamp = message.header.stamp
        stamp_seconds = (
            float(stamp.sec)
            + float(stamp.nanosec) / 1_000_000_000.0
        )

        snapshot = {
            'frame_id': str(message.header.frame_id),
            'stamp_seconds': stamp_seconds,
            'angle_min': float(message.angle_min),
            'angle_max': float(message.angle_max),
            'angle_increment': float(message.angle_increment),
            'time_increment': float(message.time_increment),
            'scan_time': float(message.scan_time),
            'range_min': float(message.range_min),
            'range_max': float(message.range_max),
            'sample_count': len(ranges),
            'valid_sample_count': len(valid_ranges),
            'ranges': ranges,
        }

        received_at = self._utc_clock().isoformat()
        received_monotonic = self._monotonic_clock()

        with self._lock:
            self._scan = snapshot
            self._received_at = received_at
            self._received_monotonic = received_monotonic

    def snapshot(self):
        """Return the latest scan without exposing mutable state."""
        now = self._monotonic_clock()

        with self._lock:
            if self._scan is None:
                return {
                    'available': False,
                    'status': 'WAITING_FOR_SCAN',
                    'received_at': None,
                    'age_seconds': None,
                    'scan': None,
                }

            age = max(
                0.0,
                now - self._received_monotonic,
            )

            scan = dict(self._scan)
            scan['ranges'] = list(self._scan['ranges'])

            return {
                'available': True,
                'status': 'READY',
                'received_at': self._received_at,
                'age_seconds': age,
                'scan': scan,
            }
