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

import app as bridge
from map_telemetry import SavedMapTelemetry


def write_test_map(tmp_path):
    yaml_path = tmp_path / 'test_map.yaml'
    pgm_path = tmp_path / 'test_map.pgm'

    yaml_path.write_text(
        '\n'.join([
            'image: test_map.pgm',
            'mode: trinary',
            'resolution: 0.05',
            'origin: [-1.0, -2.0, 0.25]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.196',
            '',
        ]),
        encoding='utf-8',
    )

    # Top row: occupied, unknown.
    # Bottom row: free, occupied.
    pgm_path.write_bytes(
        b'P5\n'
        b'# test map\n'
        b'2 2\n'
        b'255\n'
        + bytes([
            0,
            128,
            255,
            0,
        ])
    )

    return yaml_path


def test_saved_map_is_loaded_and_converted(tmp_path):
    telemetry = SavedMapTelemetry(
        yaml_path=write_test_map(tmp_path),
        utc_clock=lambda: datetime(
            2026,
            8,
            5,
            tzinfo=timezone.utc,
        ),
    )

    snapshot = telemetry.snapshot()
    occupancy_map = snapshot['map']

    assert snapshot['available'] is True
    assert snapshot['status'] == 'READY'
    assert snapshot['loaded_at'] == (
        '2026-08-05T00:00:00+00:00'
    )
    assert occupancy_map['frame_id'] == 'map'
    assert occupancy_map['width'] == 2
    assert occupancy_map['height'] == 2
    assert occupancy_map['resolution'] == 0.05
    assert occupancy_map['origin'] == {
        'x': -1.0,
        'y': -2.0,
        'yaw': 0.25,
    }

    # Bottom PGM row is returned first for occupancy-grid order.
    assert occupancy_map['cells'] == [
        0,
        100,
        100,
        -1,
    ]
    assert occupancy_map['free_cell_count'] == 1
    assert occupancy_map['occupied_cell_count'] == 2
    assert occupancy_map['unknown_cell_count'] == 1


def test_snapshot_does_not_expose_mutable_state(tmp_path):
    telemetry = SavedMapTelemetry(
        yaml_path=write_test_map(tmp_path),
    )

    first = telemetry.snapshot()
    first['map']['cells'][0] = 77
    first['map']['origin']['x'] = 99.0

    second = telemetry.snapshot()

    assert second['map']['cells'][0] == 0
    assert second['map']['origin']['x'] == -1.0


def test_missing_map_is_reported_without_exception(tmp_path):
    telemetry = SavedMapTelemetry(
        yaml_path=tmp_path / 'missing.yaml',
    )

    snapshot = telemetry.snapshot()

    assert snapshot['available'] is False
    assert snapshot['status'] == 'MAP_UNAVAILABLE'
    assert snapshot['map'] is None
    assert snapshot['error']


def test_map_endpoint_reports_unavailable(monkeypatch, tmp_path):
    telemetry = SavedMapTelemetry(
        yaml_path=tmp_path / 'missing.yaml',
    )
    monkeypatch.setattr(
        bridge,
        'map_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/map')
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['telemetry']['available'] is False


def test_map_endpoint_returns_occupancy_grid(
    monkeypatch,
    tmp_path,
):
    telemetry = SavedMapTelemetry(
        yaml_path=write_test_map(tmp_path),
    )
    monkeypatch.setattr(
        bridge,
        'map_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.get('/telemetry/map')
    payload = response.get_json()

    assert response.status_code == 200
    assert response.headers[
        'Access-Control-Allow-Origin'
    ] == '*'
    assert payload['ok'] is True
    assert payload['service'] == (
        'mini_pupper_robot_bridge'
    )
    assert payload['source'] == 'validated_saved_map'
    assert payload['telemetry']['map']['cells'] == [
        0,
        100,
        100,
        -1,
    ]


def test_map_endpoint_rejects_post(monkeypatch, tmp_path):
    telemetry = SavedMapTelemetry(
        yaml_path=write_test_map(tmp_path),
    )
    monkeypatch.setattr(
        bridge,
        'map_telemetry',
        telemetry,
    )

    client = bridge.app.test_client()
    response = client.post('/telemetry/map')

    assert response.status_code == 405
