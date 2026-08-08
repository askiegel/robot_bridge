#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

from datetime import datetime
from datetime import timezone
from types import SimpleNamespace

import app as bridge
from live_mapping_telemetry import LiveMappingTelemetry


def make_grid(cells=None):
    if cells is None:
        cells = [-1, 0, 37, 100]

    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='map',
            stamp=SimpleNamespace(
                sec=100,
                nanosec=250_000_000,
            ),
        ),
        info=SimpleNamespace(
            width=2,
            height=2,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(
                    x=-1.5,
                    y=-2.0,
                    z=0.0,
                ),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                ),
            ),
        ),
        data=cells,
    )


def running_mapping():
    return {
        'running': True,
        'owned': True,
        'pid': 123,
        'state': 'RUNNING',
    }


def stopped_mapping():
    return {
        'running': False,
        'owned': False,
        'pid': None,
        'state': 'STOPPED',
    }


def test_empty_telemetry_waits_for_map():
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: 10.0,
    )

    snapshot = telemetry.snapshot()

    assert snapshot == {
        'available': False,
        'status': 'WAITING_FOR_MAP',
        'received_at': None,
        'age_seconds': None,
        'error': None,
        'map': None,
    }


def test_grid_is_serialized_for_http():
    monotonic_values = iter([10.0, 10.25])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
        utc_clock=lambda: datetime(
            2026,
            8,
            8,
            tzinfo=timezone.utc,
        ),
    )

    telemetry.update(make_grid())
    snapshot = telemetry.snapshot()
    occupancy_map = snapshot['map']

    assert snapshot['available'] is True
    assert snapshot['status'] == 'READY'
    assert snapshot['age_seconds'] == 0.25
    assert occupancy_map['frame_id'] == 'map'
    assert occupancy_map['stamp_seconds'] == 100.25
    assert occupancy_map['width'] == 2
    assert occupancy_map['height'] == 2
    assert occupancy_map['resolution'] == 0.05
    assert occupancy_map['origin'] == {
        'x': -1.5,
        'y': -2.0,
        'yaw': 0.0,
    }
    assert occupancy_map['cell_count'] == 4
    assert occupancy_map['unknown_cell_count'] == 1
    assert occupancy_map['free_cell_count'] == 1
    assert occupancy_map['occupied_cell_count'] == 1
    assert occupancy_map['probability_cell_count'] == 1
    assert occupancy_map['cells'] == [-1, 0, 37, 100]
    assert occupancy_map['encoding'] == (
        'ros_occupancy_probabilities'
    )
    assert occupancy_map['source']['authoritative'] is False


def test_snapshot_does_not_expose_mutable_cells():
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_grid())

    first = telemetry.snapshot()
    first['map']['cells'][0] = 100
    first['map']['origin']['x'] = 99.0

    second = telemetry.snapshot()

    assert second['map']['cells'][0] == -1
    assert second['map']['cells'][2] == 37
    assert second['map']['origin']['x'] == -1.5


def test_clear_discards_previous_session_grid():
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_grid())

    assert telemetry.snapshot()['available'] is True

    telemetry.clear()
    snapshot = telemetry.snapshot()

    assert snapshot['available'] is False
    assert snapshot['map'] is None
    assert snapshot['received_at'] is None
    assert snapshot['age_seconds'] is None


def test_invalid_grid_is_not_published():
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: 10.0,
    )
    telemetry.update(
        make_grid(cells=[-1, 0, 37, 101])
    )

    snapshot = telemetry.snapshot()

    assert snapshot['available'] is False
    assert snapshot['status'] == 'INVALID_MAP'
    assert snapshot['map'] is None
    assert snapshot['error'] is not None


def test_probability_counts_cover_every_cell():
    monotonic_values = iter([10.0, 10.1])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(
        make_grid(cells=[-1, 0, 37, 100])
    )

    occupancy_map = telemetry.snapshot()['map']

    assert (
        occupancy_map['unknown_cell_count']
        + occupancy_map['free_cell_count']
        + occupancy_map['probability_cell_count']
        + occupancy_map['occupied_cell_count']
        == occupancy_map['cell_count']
    )


def test_endpoint_waits_while_mapping_starts(monkeypatch):
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: 10.0,
    )

    monkeypatch.setattr(
        bridge,
        'live_mapping_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        running_mapping,
    )

    response = bridge.app.test_client().get(
        '/telemetry/mapping-map'
    )
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['runtime_active'] is True
    assert payload['telemetry']['status'] == (
        'WAITING_FOR_MAP'
    )


def test_endpoint_returns_live_grid(monkeypatch):
    monotonic_values = iter([10.0, 10.1])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_grid())

    monkeypatch.setattr(
        bridge,
        'live_mapping_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        running_mapping,
    )

    response = bridge.app.test_client().get(
        '/telemetry/mapping-map'
    )
    payload = response.get_json()

    assert response.status_code == 200
    assert response.headers[
        'Access-Control-Allow-Origin'
    ] == '*'
    assert payload['ok'] is True
    assert payload['runtime_active'] is True
    assert payload['read_only'] is True
    assert payload['authoritative'] is False
    assert payload['topic'] == '/map'
    assert payload['telemetry']['map']['width'] == 2


def test_stopped_endpoint_erases_cached_grid(monkeypatch):
    monotonic_values = iter([10.0, 10.1, 10.2])
    telemetry = LiveMappingTelemetry(
        monotonic_clock=lambda: next(monotonic_values),
    )
    telemetry.update(make_grid())

    monkeypatch.setattr(
        bridge,
        'live_mapping_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        stopped_mapping,
    )

    response = bridge.app.test_client().get(
        '/telemetry/mapping-map'
    )
    payload = response.get_json()

    assert response.status_code == 503
    assert payload['ok'] is False
    assert payload['runtime_active'] is False
    assert payload['telemetry']['status'] == (
        'MAPPING_STOPPED'
    )
    assert payload['telemetry']['map'] is None

    internal = telemetry.snapshot()
    assert internal['available'] is False
    assert internal['map'] is None


def test_endpoint_is_get_only():
    client = bridge.app.test_client()

    assert client.post(
        '/telemetry/mapping-map'
    ).status_code == 405
