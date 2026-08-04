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

from datetime import datetime
from datetime import timezone
from types import SimpleNamespace

import app as bridge
from lidar_telemetry import LidarTelemetry


def make_scan():
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='lidar_link',
            stamp=SimpleNamespace(
                sec=100,
                nanosec=250_000_000,
            ),
        ),
        angle_min=-3.14,
        angle_max=3.14,
        angle_increment=0.01,
        time_increment=0.0002,
        scan_time=0.1,
        range_min=0.03,
        range_max=12.0,
        ranges=[
            1.0,
            float('inf'),
            float('nan'),
            2.5,
        ],
    )


def test_empty_telemetry_waits_for_scan():
    telemetry = LidarTelemetry(
        monotonic_clock=lambda: 10.0,
    )

    snapshot = telemetry.snapshot()

    assert snapshot['available'] is False
    assert snapshot['status'] == 'WAITING_FOR_SCAN'
    assert snapshot['scan'] is None


def test_scan_is_serialized_for_http():
    monotonic_values = iter([10.0, 10.25])
    telemetry = LidarTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
        utc_clock=lambda: datetime(
            2026,
            8,
            4,
            tzinfo=timezone.utc,
        ),
    )

    telemetry.update(make_scan())
    snapshot = telemetry.snapshot()
    scan = snapshot['scan']

    assert snapshot['available'] is True
    assert snapshot['status'] == 'READY'
    assert snapshot['age_seconds'] == 0.25
    assert scan['frame_id'] == 'lidar_link'
    assert scan['stamp_seconds'] == 100.25
    assert scan['sample_count'] == 4
    assert scan['valid_sample_count'] == 2
    assert scan['ranges'] == [1.0, None, None, 2.5]


def test_lidar_endpoint_reports_waiting(monkeypatch):
    telemetry = LidarTelemetry(
        monotonic_clock=lambda: 10.0,
    )
    monkeypatch.setattr(
        bridge,
        'lidar_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/lidar')
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['telemetry']['available'] is False


def test_lidar_endpoint_returns_scan(monkeypatch):
    monotonic_values = iter([10.0, 10.1])
    telemetry = LidarTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_scan())

    monkeypatch.setattr(
        bridge,
        'lidar_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/lidar')
    payload = response.get_json()

    assert response.status_code == 200
    assert response.headers['Access-Control-Allow-Origin'] == '*'
    assert payload['ok'] is True
    assert payload['service'] == 'mini_pupper_robot_bridge'
    assert payload['telemetry']['scan']['sample_count'] == 4
