#!/usr/bin/env python3

#
# SPDX-License-Identifier: Apache-2.0
#

import hashlib
import json
from pathlib import Path

import app as bridge
import pytest

from map_promotion import (
    MapPromotion,
    MapPromotionConflictError,
    MapPromotionError,
)


CANDIDATE = (
    'mayday_map_candidate_20260808T161300Z'
)
TIMESTAMP = '2026-08-08T19:30:45+00:00'


def digest(path):
    return hashlib.sha256(
        Path(path).read_bytes()
    ).hexdigest()


def write_manifest(directory, names):
    lines = [
        f'{digest(directory / name)}  {name}'
        for name in names
    ]

    (directory / 'SHA256SUMS').write_text(
        '\n'.join(lines) + '\n',
        encoding='utf-8',
    )


def create_map(directory, base_name):
    directory.mkdir(parents=True)

    pbstream = f'{base_name}.pbstream'
    yaml = f'{base_name}.yaml'
    pgm = f'{base_name}.pgm'

    (directory / pbstream).write_bytes(
        b'pbstream-' + base_name.encode()
    )
    (directory / pgm).write_bytes(
        b'P5\n2 2\n255\n'
        + bytes((0, 205, 254, 255))
    )
    (directory / yaml).write_text(
        '\n'.join((
            f'image: {pgm}',
            'resolution: 0.050000',
            'origin: [0.0, 0.0, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.196',
            '',
        )),
        encoding='utf-8',
    )

    write_manifest(
        directory,
        (pbstream, yaml, pgm),
    )


class ReloadTelemetry:
    def __init__(self, failures=0):
        self.failures = failures
        self.reload_count = 0

    def reload(self):
        self.reload_count += 1

        if self.reload_count <= self.failures:
            raise ValueError(
                'Synthetic telemetry load failure.'
            )

        return {
            'available': True,
            'status': 'READY',
        }


class ReviewTelemetry:
    def __init__(self, root):
        self.root = root

    def snapshot(self):
        return {
            'available': True,
            'read_only': True,
            'promotion_enabled': False,
            'candidates': [
                {
                    'name': CANDIDATE,
                    'directory': str(
                        self.root / CANDIDATE
                    ),
                    'classification': 'REVIEW_READY',
                    'review_ready': True,
                    'checksums_valid': True,
                    'image_reference_valid': True,
                    'promoted': False,
                    'comparison': {
                        'same_frame': True,
                        'same_resolution': True,
                    },
                },
            ],
        }


def make_control(
    tmp_path,
    state=None,
    validated_telemetry=None,
):
    root = tmp_path / 'maps'
    validated = (
        root / MapPromotion.VALIDATED_NAME
    )
    candidate = root / CANDIDATE

    create_map(
        validated,
        MapPromotion.VALIDATED_NAME,
    )
    create_map(
        candidate,
        CANDIDATE,
    )

    runtime = state or {
        'mapping': {
            'running': False,
            'owned': False,
            'pid': None,
        },
        'localization': {
            'running': False,
            'owned': False,
            'pid': None,
        },
    }

    control = MapPromotion(
        map_root=root,
        candidate_map_telemetry=(
            ReviewTelemetry(root)
        ),
        validated_map_telemetry=(
            validated_telemetry
        ),
        runtime_state_provider=lambda: runtime,
    )

    return control, root, validated, candidate


def test_promotion_creates_backup_and_canonical_map(
    tmp_path,
):
    (
        control,
        root,
        validated,
        candidate,
    ) = make_control(tmp_path)

    original_validated = {
        path.name: path.read_bytes()
        for path in validated.iterdir()
        if path.is_file()
    }
    original_candidate = {
        path.name: path.read_bytes()
        for path in candidate.iterdir()
        if path.is_file()
    }

    result = control.promote(
        CANDIDATE,
        MapPromotion.CONFIRMATION,
        TIMESTAMP,
    )

    backup = Path(result['backup_directory'])

    assert result['promoted'] is True
    assert result['candidate_preserved'] is True
    assert backup.is_dir()
    assert validated.is_dir()

    for name, contents in original_validated.items():
        if name == 'SHA256SUMS':
            continue

        assert (
            backup / name
        ).read_bytes() == contents

    assert {
        path.name: path.read_bytes()
        for path in candidate.iterdir()
        if path.is_file()
    } == original_candidate

    canonical = MapPromotion.VALIDATED_NAME

    assert (
        validated / f'{canonical}.pbstream'
    ).read_bytes() == (
        candidate / f'{CANDIDATE}.pbstream'
    ).read_bytes()

    assert (
        f'image: {canonical}.pgm'
        in (
            validated / f'{canonical}.yaml'
        ).read_text(encoding='utf-8')
    )

    metadata = json.loads(
        (
            validated / 'PROMOTION_METADATA.json'
        ).read_text(encoding='utf-8')
    )

    assert metadata['promoted'] is True
    assert metadata['source_candidate'] == CANDIDATE
    assert metadata['candidate_preserved'] is True
    assert metadata['motion_enabled'] is False


def test_successful_promotion_reloads_telemetry(
    tmp_path,
):
    telemetry = ReloadTelemetry()

    control, _, _, _ = make_control(
        tmp_path,
        validated_telemetry=telemetry,
    )

    control.promote(
        CANDIDATE,
        MapPromotion.CONFIRMATION,
        TIMESTAMP,
    )

    assert telemetry.reload_count == 1


def test_telemetry_failure_rolls_back_validated_map(
    tmp_path,
):
    telemetry = ReloadTelemetry(failures=1)

    control, _, validated, candidate = make_control(
        tmp_path,
        validated_telemetry=telemetry,
    )

    canonical = MapPromotion.VALIDATED_NAME
    original_pbstream = (
        validated / f'{canonical}.pbstream'
    ).read_bytes()
    candidate_pbstream = (
        candidate / f'{CANDIDATE}.pbstream'
    ).read_bytes()

    assert original_pbstream != candidate_pbstream

    with pytest.raises(
        MapPromotionError,
        match='rolled back',
    ):
        control.promote(
            CANDIDATE,
            MapPromotion.CONFIRMATION,
            TIMESTAMP,
        )

    assert telemetry.reload_count == 2
    assert (
        validated / f'{canonical}.pbstream'
    ).read_bytes() == original_pbstream


def test_invalid_candidate_name_is_rejected(tmp_path):
    control, _, _, _ = make_control(tmp_path)

    with pytest.raises(MapPromotionError):
        control.promote(
            '../../unsafe',
            MapPromotion.CONFIRMATION,
            TIMESTAMP,
        )


def test_confirmation_is_required(tmp_path):
    control, _, _, _ = make_control(tmp_path)

    with pytest.raises(MapPromotionError):
        control.promote(
            CANDIDATE,
            'wrong',
            TIMESTAMP,
        )


def test_running_mapping_blocks_promotion(tmp_path):
    state = {
        'mapping': {
            'running': True,
            'owned': True,
            'pid': 123,
        },
        'localization': {
            'running': False,
            'owned': False,
            'pid': None,
        },
    }

    control, _, _, _ = make_control(
        tmp_path,
        state=state,
    )

    with pytest.raises(
        MapPromotionConflictError
    ):
        control.promote(
            CANDIDATE,
            MapPromotion.CONFIRMATION,
            TIMESTAMP,
        )


def test_snapshot_has_no_motion_or_planning(tmp_path):
    control, _, _, _ = make_control(tmp_path)
    snapshot = control.snapshot()

    assert snapshot['promotion_enabled'] is True
    assert snapshot['motion_enabled'] is False
    assert snapshot['planning_enabled'] is False
    assert snapshot[
        'mapping_required_stopped'
    ] is True
    assert snapshot[
        'localization_required_stopped'
    ] is True


class RoutePromotion:
    CONFIRMATION = MapPromotion.CONFIRMATION

    def snapshot(self):
        return {
            'promotion_enabled': True,
            'motion_enabled': False,
        }

    def promote(
        self,
        candidate_name,
        confirmation,
        timestamp,
    ):
        assert candidate_name == CANDIDATE
        assert confirmation == self.CONFIRMATION

        return {
            'promoted': True,
            'candidate_name': candidate_name,
            'timestamp': timestamp,
            'motion_enabled': False,
        }


def stopped_snapshot():
    return {
        'running': False,
        'owned': False,
        'pid': None,
    }


def test_promotion_endpoint_is_post_only(monkeypatch):
    monkeypatch.setattr(
        bridge,
        'map_promotion',
        RoutePromotion(),
    )

    client = bridge.app.test_client()

    assert client.get(
        '/map/promote-candidate'
    ).status_code == 405


def test_promotion_endpoint_requires_exact_body(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )
    monkeypatch.setattr(
        bridge,
        'map_promotion',
        RoutePromotion(),
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        stopped_snapshot,
    )
    monkeypatch.setattr(
        bridge.localization_control,
        'snapshot',
        stopped_snapshot,
    )

    client = bridge.app.test_client()

    response = client.post(
        '/map/promote-candidate',
        json={
            'candidate_name': CANDIDATE,
            'confirmation': (
                MapPromotion.CONFIRMATION
            ),
            'path': '/tmp/unsafe',
        },
    )

    assert response.status_code == 400


def test_promotion_endpoint_publishes_zero(
    monkeypatch,
):
    calls = []

    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: (
            calls.append('zero')
            or {'ok': True}
        ),
    )
    monkeypatch.setattr(
        bridge,
        'map_promotion',
        RoutePromotion(),
    )
    monkeypatch.setattr(
        bridge.mapping_control,
        'snapshot',
        stopped_snapshot,
    )
    monkeypatch.setattr(
        bridge.localization_control,
        'snapshot',
        stopped_snapshot,
    )

    response = bridge.app.test_client().post(
        '/map/promote-candidate',
        json={
            'candidate_name': CANDIDATE,
            'confirmation': (
                MapPromotion.CONFIRMATION
            ),
        },
    )

    assert response.status_code == 201
    assert calls == ['zero']
    assert response.get_json()['promotion'][
        'promoted'
    ] is True
