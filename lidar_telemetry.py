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
import uuid
from datetime import datetime
from datetime import timezone
from local_motion_guard import MAX_SCAN_SAMPLES


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
        self._producer_session = str(uuid.uuid4())
        self._acquisition_sequence = 0  # Distinct source scans, not snapshot reads.
        self._source_stamp_ns = None
        self._currentness_fault = None

    @staticmethod
    def _finite_or_none(value):
        value = float(value)

        if math.isfinite(value):
            return value

        return None

    def _invalidate_source(self, reason):
        # An active command must observe even a fault followed by a fresh scan
        # between two polls. This is a local intake epoch, not driver identity.
        self._producer_session = str(uuid.uuid4())
        self._currentness_fault = reason

    def update(self, message):
        """Record one immutable JSON-compatible scan snapshot."""
        # Bound conversion, equality and copying before a scan can enter the
        # shared cache. Rejected data never refreshes the last usable receipt.
        if not 1 <= len(message.ranges) <= MAX_SCAN_SAMPLES:
            with self._lock:
                self._invalidate_source('active_lidar_geometry_invalid')
            return
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
        if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
                or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
            with self._lock:
                self._invalidate_source('active_lidar_source_stamp_invalid')
            return
        source_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
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
            if self._source_stamp_ns is not None:
                if source_stamp_ns < self._source_stamp_ns:
                    self._invalidate_source('active_lidar_source_regressed')
                    return
                if source_stamp_ns == self._source_stamp_ns:
                    # Repeated callbacks cannot rejuvenate frozen source data.
                    if snapshot != self._scan:
                        self._invalidate_source('active_lidar_source_stamp_reused')
                    return
            self._scan = snapshot
            self._received_at = received_at
            self._received_monotonic = received_monotonic
            self._source_stamp_ns = source_stamp_ns
            self._acquisition_sequence += 1
            self._currentness_fault = None

    def snapshot(self):
        """Bounded cache read; never wait for sensor acquisition/cache writers."""
        now = self._monotonic_clock()

        if not self._lock.acquire(blocking=False):
            return {'available': False, 'status': 'CACHE_BUSY', 'received_at': None,
                    'age_seconds': None, 'scan': None}
        try:
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
                'producer_session': self._producer_session,
                'acquisition_sequence': self._acquisition_sequence,
                'source_stamp_ns': self._source_stamp_ns,
                'received_monotonic_seconds': self._received_monotonic,
                'currentness_fault': self._currentness_fault,
            }
        finally:
            self._lock.release()
