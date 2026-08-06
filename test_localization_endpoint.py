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
from datetime import datetime
from datetime import timezone
from types import SimpleNamespace

import app as bridge
from localization_telemetry import LocalizationTelemetry


def make_pose(yaw=0.5):
    covariance = [0.0] * 36
    covariance[0] = 0.04
    covariance[7] = 0.09
    covariance[35] = 0.01

    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='map',
            stamp=SimpleNamespace(
                sec=200,
                nanosec=500_000_000,
            ),
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=1.25,
                    y=-0.75,
                    z=0.0,
                ),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
            covariance=covariance,
        ),
    )


def test_empty_telemetry_waits_for_localization():
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: 10.0,
    )

    snapshot = telemetry.snapshot()

    assert snapshot['available'] is False
    assert snapshot['status'] == (
        'WAITING_FOR_LOCALIZATION'
    )
    assert snapshot['pose'] is None


def test_pose_is_serialized_for_http():
    monotonic_values = iter([10.0, 10.25])
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
        utc_clock=lambda: datetime(
            2026,
            8,
            5,
            tzinfo=timezone.utc,
        ),
    )

    telemetry.update(make_pose())
    snapshot = telemetry.snapshot()
    pose = snapshot['pose']

    assert snapshot['available'] is True
    assert snapshot['status'] == 'READY'
    assert snapshot['age_seconds'] == 0.25
    assert pose['frame_id'] == 'map'
    assert pose['stamp_seconds'] == 200.5
    assert pose['position'] == {
        'x': 1.25,
        'y': -0.75,
        'z': 0.0,
    }
    assert math.isclose(
        pose['yaw_radians'],
        0.5,
        abs_tol=1e-12,
    )
    assert math.isclose(
        pose['yaw_degrees'],
        math.degrees(0.5),
        abs_tol=1e-12,
    )
    assert pose['uncertainty'] == {
        'x_variance': 0.04,
        'y_variance': 0.09,
        'yaw_variance': 0.01,
        'x_standard_deviation': 0.2,
        'y_standard_deviation': 0.3,
        'yaw_standard_deviation_radians': 0.1,
    }


def test_snapshot_does_not_expose_mutable_state():
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )

    telemetry.update(make_pose())
    first = telemetry.snapshot()
    first['pose']['position']['x'] = 99.0
    first['pose']['covariance'][0] = 99.0

    second = telemetry.snapshot()

    assert second['pose']['position']['x'] == 1.25
    assert second['pose']['covariance'][0] == 0.04


def test_zero_quaternion_is_rejected():
    message = make_pose()
    message.pose.pose.orientation.z = 0.0
    message.pose.pose.orientation.w = 0.0

    telemetry = LocalizationTelemetry()

    try:
        telemetry.update(message)
    except ValueError as error:
        assert 'zero magnitude' in str(error)
    else:
        raise AssertionError(
            'Zero quaternion should be rejected.'
        )


def test_localization_cache_clear_discards_pose():
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_pose())

    assert telemetry.snapshot()['available'] is True

    telemetry.clear()
    snapshot = telemetry.snapshot()

    assert snapshot == {
        'available': False,
        'status': 'WAITING_FOR_LOCALIZATION',
        'received_at': None,
        'age_seconds': None,
        'pose': None,
    }


def test_localization_endpoint_reports_waiting(monkeypatch):
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: 10.0,
    )
    monkeypatch.setattr(
        bridge,
        'localization_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge,
        'localization_runtime_active',
        lambda: True,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/localization')
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['topic'] == '/amcl_pose'
    assert payload['telemetry']['available'] is False


def test_localization_endpoint_returns_pose(monkeypatch):
    monotonic_values = iter([10.0, 10.1])
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_pose())

    monkeypatch.setattr(
        bridge,
        'localization_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge,
        'localization_runtime_active',
        lambda: True,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/localization')
    payload = response.get_json()

    assert response.status_code == 200
    assert response.headers[
        'Access-Control-Allow-Origin'
    ] == '*'
    assert payload['ok'] is True
    assert payload['runtime_active'] is True
    assert payload['service'] == (
        'mini_pupper_robot_bridge'
    )
    assert payload['topic'] == '/amcl_pose'
    assert payload['telemetry']['pose']['frame_id'] == 'map'


def test_cached_pose_is_hidden_when_localization_stops(
    monkeypatch,
):
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LocalizationTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_pose())

    monkeypatch.setattr(
        bridge,
        'localization_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge,
        'localization_runtime_active',
        lambda: False,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/localization')
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['runtime_active'] is False
    assert payload['telemetry']['available'] is False
    assert payload['telemetry']['status'] == (
        'LOCALIZATION_STOPPED'
    )
    assert payload['telemetry']['pose'] is None
    assert payload['telemetry']['received_at'] is None
    assert payload['telemetry']['age_seconds'] is None

    internal_snapshot = telemetry.snapshot()
    assert internal_snapshot['available'] is False
    assert internal_snapshot['pose'] is None
    assert internal_snapshot['received_at'] is None


def test_localization_endpoint_rejects_post(monkeypatch):
    telemetry = LocalizationTelemetry()
    monkeypatch.setattr(
        bridge,
        'localization_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.post('/telemetry/localization')

    assert response.status_code == 405
