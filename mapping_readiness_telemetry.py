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

import threading
import time
from datetime import datetime
from datetime import timezone


class MappingReadinessTelemetry:
    """Track live Cartographer candidate-export readiness."""

    def __init__(
        self,
        minimum_submaps,
        minimum_mature_submaps,
        minimum_mature_version,
        monotonic_clock=None,
        utc_clock=None,
    ):
        self._lock = threading.Lock()
        self._minimum_submaps = int(minimum_submaps)
        self._minimum_mature_submaps = int(
            minimum_mature_submaps
        )
        self._minimum_mature_version = int(
            minimum_mature_version
        )
        self._monotonic_clock = (
            monotonic_clock or time.monotonic
        )
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )
        self._readiness = None
        self._received_at = None
        self._received_monotonic = None
        self._error = None

        if self._minimum_submaps <= 0:
            raise ValueError(
                'Minimum submap count must be positive.'
            )

        if self._minimum_mature_submaps <= 0:
            raise ValueError(
                'Minimum mature-submap count must be '
                'positive.'
            )

        if self._minimum_mature_version < 0:
            raise ValueError(
                'Minimum mature version cannot be negative.'
            )

    def requirements(self):
        """Return immutable candidate-export requirements."""
        return {
            'minimum_submap_count': (
                self._minimum_submaps
            ),
            'minimum_mature_submap_count': (
                self._minimum_mature_submaps
            ),
            'minimum_mature_version': (
                self._minimum_mature_version
            ),
        }

    def update(self, message):
        """Record the newest active Cartographer trajectory."""
        entries = []
        error = None

        try:
            for submap in message.submap:
                trajectory_id = int(
                    submap.trajectory_id
                )
                index = int(submap.submap_index)
                version = int(submap.submap_version)
                frozen = bool(submap.is_frozen)

                if trajectory_id < 0:
                    raise ValueError(
                        'Trajectory ID cannot be negative.'
                    )

                if index < 0:
                    raise ValueError(
                        'Submap index cannot be negative.'
                    )

                if version < 0:
                    raise ValueError(
                        'Submap version cannot be negative.'
                    )

                entries.append({
                    'trajectory_id': trajectory_id,
                    'index': index,
                    'version': version,
                    'is_frozen': frozen,
                })

        except Exception as exc:
            error = str(exc)

        if error is not None:
            with self._lock:
                self._error = error
            return

        trajectory_ids = sorted({
            entry['trajectory_id']
            for entry in entries
        })

        active_ids = sorted({
            entry['trajectory_id']
            for entry in entries
            if not entry['is_frozen']
        })

        trajectory_id = (
            active_ids[-1]
            if active_ids
            else (
                trajectory_ids[-1]
                if trajectory_ids
                else None
            )
        )

        selected_by_index = {}

        for entry in entries:
            if entry['trajectory_id'] != trajectory_id:
                continue

            existing = selected_by_index.get(
                entry['index']
            )

            if (
                existing is None
                or entry['version'] > existing['version']
            ):
                selected_by_index[
                    entry['index']
                ] = entry

        selected = [
            selected_by_index[index]
            for index in sorted(selected_by_index)
        ]

        submaps = [
            {
                'index': entry['index'],
                'version': entry['version'],
                'is_frozen': entry['is_frozen'],
            }
            for entry in selected
        ]

        mature_count = sum(
            1
            for submap in submaps
            if submap['version']
            >= self._minimum_mature_version
        )
        submap_count = len(submaps)
        ready = (
            submap_count >= self._minimum_submaps
            and mature_count
            >= self._minimum_mature_submaps
        )

        if ready:
            status = 'READY_TO_SAVE'
        elif submap_count == 0:
            status = 'WAITING_FOR_SUBMAPS'
        else:
            status = 'BUILDING_SUBMAPS'

        readiness = {
            'available': True,
            'status': status,
            'ready': ready,
            'trajectory_id': trajectory_id,
            'submap_count': submap_count,
            'mature_submap_count': mature_count,
            **self.requirements(),
            'submap_progress': min(
                1.0,
                submap_count / self._minimum_submaps,
            ),
            'mature_submap_progress': min(
                1.0,
                mature_count
                / self._minimum_mature_submaps,
            ),
            'submaps': submaps,
        }

        received_at = self._utc_clock().isoformat()
        received_monotonic = self._monotonic_clock()

        with self._lock:
            self._readiness = readiness
            self._received_at = received_at
            self._received_monotonic = (
                received_monotonic
            )
            self._error = None

    def clear(self):
        """Discard readiness from a previous session."""
        with self._lock:
            self._readiness = None
            self._received_at = None
            self._received_monotonic = None
            self._error = None

    def snapshot(self, runtime_active):
        """Return current readiness without ROS side effects."""
        requirements = self.requirements()

        with self._lock:
            if not runtime_active:
                return {
                    'available': False,
                    'status': 'MAPPING_STOPPED',
                    'ready': False,
                    'received_at': None,
                    'age_seconds': None,
                    'error': None,
                    'trajectory_id': None,
                    'submap_count': 0,
                    'mature_submap_count': 0,
                    **requirements,
                    'submap_progress': 0.0,
                    'mature_submap_progress': 0.0,
                    'submaps': [],
                }

            if self._readiness is None:
                return {
                    'available': False,
                    'status': (
                        'INVALID_SUBMAP_LIST'
                        if self._error
                        else 'WAITING_FOR_SUBMAPS'
                    ),
                    'ready': False,
                    'received_at': None,
                    'age_seconds': None,
                    'error': self._error,
                    'trajectory_id': None,
                    'submap_count': 0,
                    'mature_submap_count': 0,
                    **requirements,
                    'submap_progress': 0.0,
                    'mature_submap_progress': 0.0,
                    'submaps': [],
                }

            now = self._monotonic_clock()
            result = dict(self._readiness)
            result['submaps'] = [
                dict(submap)
                for submap in self._readiness['submaps']
            ]
            result['received_at'] = self._received_at
            result['age_seconds'] = max(
                0.0,
                now - self._received_monotonic,
            )
            result['error'] = None
            return result
