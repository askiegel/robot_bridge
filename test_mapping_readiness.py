#!/usr/bin/env python3

from datetime import datetime
from datetime import timezone
from types import SimpleNamespace

import app as bridge
from mapping_readiness_telemetry import (
    MappingReadinessTelemetry,
)


def make_entry(
    trajectory_id,
    index,
    version,
    frozen=False,
):
    return SimpleNamespace(
        trajectory_id=trajectory_id,
        submap_index=index,
        submap_version=version,
        is_frozen=frozen,
    )


def make_message(entries):
    return SimpleNamespace(submap=entries)


def make_telemetry(monotonic_values=None):
    values = iter(monotonic_values or [10.0])

    return MappingReadinessTelemetry(
        minimum_submaps=3,
        minimum_mature_submaps=2,
        minimum_mature_version=100,
        monotonic_clock=lambda: next(values),
        utc_clock=lambda: datetime(
            2026,
            8,
            8,
            22,
            0,
            tzinfo=timezone.utc,
        ),
    )


def running_mapping():
    return {
        'running': True,
        'owned': True,
        'pid': 1234,
        'state': 'RUNNING',
    }


def stopped_mapping():
    return {
        'running': False,
        'owned': False,
        'pid': None,
        'state': 'STOPPED',
    }


def mature_message():
    return make_message([
        make_entry(0, 0, 180),
        make_entry(0, 1, 106),
        make_entry(0, 2, 16),
    ])


def test_waits_before_first_submap():
    telemetry = make_telemetry()

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['available'] is False
    assert snapshot['status'] == 'WAITING_FOR_SUBMAPS'
    assert snapshot['ready'] is False
    assert snapshot['submap_count'] == 0
    assert snapshot['mature_submap_count'] == 0


def test_reports_building_progress():
    telemetry = make_telemetry([10.0, 10.5])
    telemetry.update(make_message([
        make_entry(0, 0, 180),
        make_entry(0, 1, 35),
    ]))

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['available'] is True
    assert snapshot['status'] == 'BUILDING_SUBMAPS'
    assert snapshot['ready'] is False
    assert snapshot['trajectory_id'] == 0
    assert snapshot['submap_count'] == 2
    assert snapshot['mature_submap_count'] == 1
    assert snapshot['submap_progress'] == 2 / 3
    assert snapshot['mature_submap_progress'] == 1 / 2
    assert snapshot['submaps'] == [
        {
            'index': 0,
            'version': 180,
            'is_frozen': False,
        },
        {
            'index': 1,
            'version': 35,
            'is_frozen': False,
        },
    ]


def test_reports_ready_to_save():
    telemetry = make_telemetry([10.0, 10.25])
    telemetry.update(mature_message())

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['available'] is True
    assert snapshot['status'] == 'READY_TO_SAVE'
    assert snapshot['ready'] is True
    assert snapshot['submap_count'] == 3
    assert snapshot['mature_submap_count'] == 2
    assert snapshot['minimum_submap_count'] == 3
    assert snapshot['minimum_mature_submap_count'] == 2
    assert snapshot['minimum_mature_version'] == 100
    assert snapshot['age_seconds'] == 0.25


def test_selects_newest_active_trajectory():
    telemetry = make_telemetry([10.0, 10.1])
    telemetry.update(make_message([
        make_entry(0, 0, 180, frozen=True),
        make_entry(0, 1, 180, frozen=True),
        make_entry(1, 0, 120),
        make_entry(1, 1, 30),
    ]))

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['trajectory_id'] == 1
    assert snapshot['submap_count'] == 2
    assert snapshot['mature_submap_count'] == 1


def test_keeps_highest_version_per_submap():
    telemetry = make_telemetry([10.0, 10.1])
    telemetry.update(make_message([
        make_entry(0, 0, 25),
        make_entry(0, 0, 110),
        make_entry(0, 1, 20),
    ]))

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['submaps'][0]['index'] == 0
    assert snapshot['submaps'][0]['version'] == 110
    assert snapshot['mature_submap_count'] == 1


def test_stopped_snapshot_exposes_no_stale_progress():
    telemetry = make_telemetry([10.0])
    telemetry.update(mature_message())

    snapshot = telemetry.snapshot(runtime_active=False)

    assert snapshot['available'] is False
    assert snapshot['status'] == 'MAPPING_STOPPED'
    assert snapshot['ready'] is False
    assert snapshot['submap_count'] == 0
    assert snapshot['mature_submap_count'] == 0
    assert snapshot['submaps'] == []


def test_clear_discards_previous_session():
    telemetry = make_telemetry([10.0])
    telemetry.update(mature_message())
    telemetry.clear()

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['status'] == 'WAITING_FOR_SUBMAPS'
    assert snapshot['ready'] is False
    assert snapshot['submap_count'] == 0


def test_invalid_submap_is_blocked():
    telemetry = make_telemetry()
    telemetry.update(make_message([
        make_entry(0, 0, -1),
    ]))

    snapshot = telemetry.snapshot(runtime_active=True)

    assert snapshot['available'] is False
    assert snapshot['status'] == 'INVALID_SUBMAP_LIST'
    assert snapshot['ready'] is False
    assert 'negative' in snapshot['error']


def test_mapping_status_includes_live_readiness(
    monkeypatch,
):
    telemetry = make_telemetry([10.0, 10.2])
    telemetry.update(mature_message())

    monkeypatch.setattr(
        bridge,
        'mapping_readiness_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        running_mapping,
    )

    response = bridge.app.test_client().get(
        '/mapping/status'
    )
    payload = response.get_json()
    readiness = payload['mapping']['readiness']

    assert response.status_code == 200
    assert payload['ok'] is True
    assert readiness['status'] == 'READY_TO_SAVE'
    assert readiness['ready'] is True
    assert readiness['submap_count'] == 3
    assert readiness['mature_submap_count'] == 2


def test_stopped_status_clears_readiness(monkeypatch):
    telemetry = make_telemetry([10.0])
    telemetry.update(mature_message())

    monkeypatch.setattr(
        bridge,
        'mapping_readiness_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        stopped_mapping,
    )

    response = bridge.app.test_client().get(
        '/mapping/status'
    )
    readiness = response.get_json()[
        'mapping'
    ]['readiness']

    assert response.status_code == 200
    assert readiness['status'] == 'MAPPING_STOPPED'
    assert readiness['ready'] is False
    assert readiness['submap_count'] == 0


def test_mapping_status_is_get_only():
    response = bridge.app.test_client().post(
        '/mapping/status'
    )

    assert response.status_code == 405
