#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import hashlib
import json
from datetime import datetime
from datetime import timezone
from pathlib import Path

import app as bridge
from candidate_map_telemetry import CandidateMapTelemetry
from map_telemetry import SavedMapTelemetry


FIXED_TIME = datetime(
    2026,
    8,
    7,
    23,
    50,
    tzinfo=timezone.utc,
)


def write_map(
    directory,
    name,
    include_mode=True,
    image_value=None,
):
    directory.mkdir(parents=True, exist_ok=True)

    image_name = f'{name}.pgm'
    yaml_path = directory / f'{name}.yaml'
    image_path = directory / image_name

    image_path.write_bytes(
        b'P5\n2 2\n255\n'
        + bytes([255, 0, 205, 255])
    )

    lines = [
        (
            'image: '
            + (
                image_value
                if image_value is not None
                else image_name
            )
        ),
    ]

    if include_mode:
        lines.append('mode: trinary')

    lines.extend([
        'resolution: 0.05',
        'origin: [-1.0, -2.0, 0.0]',
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
    ])

    yaml_path.write_text(
        '\n'.join(lines) + '\n',
        encoding='utf-8',
    )

    return yaml_path, image_path


def sha256(path):
    return hashlib.sha256(
        path.read_bytes()
    ).hexdigest()


def create_candidate(
    root,
    name,
    image_value=None,
    corrupt_checksum=False,
):
    directory = root / name
    yaml_path, image_path = write_map(
        directory,
        name,
        include_mode=False,
        image_value=image_value,
    )
    pbstream_path = directory / f'{name}.pbstream'
    pbstream_path.write_bytes(b'pbstream')

    metadata_path = (
        directory / 'CANDIDATE_METADATA.json'
    )
    metadata_path.write_text(
        json.dumps({
            'candidate_name': name,
            'status': 'CANDIDATE_REVIEW_REQUIRED',
            'promoted': False,
            'validated_map_changed': False,
            'trajectory_id': 0,
            'resolution': 0.05,
            'frame_id': 'map',
            'submap_readiness': {
                'submap_count': 3,
                'mature_submap_count': 2,
                'minimum_mature_version': 100,
            },
        }),
        encoding='utf-8',
    )

    artifacts = (
        pbstream_path,
        yaml_path,
        image_path,
        metadata_path,
    )

    lines = []

    for artifact in artifacts:
        digest = sha256(artifact)

        if (
            corrupt_checksum
            and artifact == yaml_path
        ):
            digest = '0' * 64

        lines.append(f'{digest}  {artifact.name}')

    (
        directory / 'SHA256SUMS'
    ).write_text(
        '\n'.join(lines) + '\n',
        encoding='utf-8',
    )

    return directory


def make_telemetry(tmp_path):
    validated_directory = tmp_path / 'validated'
    validated_yaml, _ = write_map(
        validated_directory,
        'validated',
    )

    validated = SavedMapTelemetry(
        yaml_path=validated_yaml,
        utc_clock=lambda: FIXED_TIME,
    )

    root = tmp_path / 'maps'
    root.mkdir()

    telemetry = CandidateMapTelemetry(
        map_root=root,
        validated_map_telemetry=validated,
        utc_clock=lambda: FIXED_TIME,
    )

    return telemetry, root


def test_cartographer_yaml_defaults_to_trinary(
    tmp_path,
):
    directory = tmp_path / 'candidate'
    yaml_path, _ = write_map(
        directory,
        'candidate',
        include_mode=False,
    )

    snapshot = SavedMapTelemetry(
        yaml_path=yaml_path,
        utc_clock=lambda: FIXED_TIME,
    ).snapshot()

    assert snapshot['available'] is True
    assert snapshot['map']['width'] == 2
    assert snapshot['map']['height'] == 2


def test_review_ready_candidate_is_loaded(tmp_path):
    telemetry, root = make_telemetry(tmp_path)
    name = 'mayday_map_candidate_20260807T235000Z'

    create_candidate(root, name)
    snapshot = telemetry.snapshot()
    candidate = snapshot['candidates'][0]

    assert snapshot['available'] is True
    assert snapshot['candidate_count'] == 1
    assert snapshot['review_ready_count'] == 1
    assert snapshot['invalid_count'] == 0
    assert snapshot['read_only'] is True
    assert snapshot['promotion_enabled'] is False
    assert candidate['classification'] == 'REVIEW_READY'
    assert candidate['review_ready'] is True
    assert candidate['map']['cells'] == [-1, 0, 0, 100]
    assert candidate['map_summary']['width'] == 2
    assert candidate['checksums_valid'] is True
    assert candidate['image_reference_valid'] is True


def test_absolute_image_reference_is_invalid(tmp_path):
    telemetry, root = make_telemetry(tmp_path)
    name = 'mayday_map_candidate_20260807T235001Z'

    create_candidate(
        root,
        name,
        image_value=(
            '/tmp/.candidate.partial/'
            f'{name}.pgm'
        ),
    )

    candidate = telemetry.snapshot()['candidates'][0]

    assert candidate['classification'] == (
        'INVALID_IMAGE_REFERENCE'
    )
    assert candidate['review_ready'] is False
    assert candidate['image_reference_valid'] is False
    assert candidate['map'] is None


def test_checksum_failure_is_invalid(tmp_path):
    telemetry, root = make_telemetry(tmp_path)
    name = 'mayday_map_candidate_20260807T235002Z'

    create_candidate(
        root,
        name,
        corrupt_checksum=True,
    )

    candidate = telemetry.snapshot()['candidates'][0]

    assert candidate['classification'] == 'INVALID_CHECKSUM'
    assert candidate['checksums_valid'] is False
    assert candidate['map'] is None


def test_snapshot_refreshes_candidate_inventory(
    tmp_path,
):
    telemetry, root = make_telemetry(tmp_path)

    first = telemetry.snapshot()
    assert first['candidate_count'] == 0

    create_candidate(
        root,
        'mayday_map_candidate_20260807T235003Z',
    )

    second = telemetry.snapshot()

    assert second['candidate_count'] == 1
    assert second['review_ready_count'] == 1


def test_candidate_comparison_uses_validated_map(
    tmp_path,
):
    telemetry, root = make_telemetry(tmp_path)
    name = 'mayday_map_candidate_20260807T235004Z'

    create_candidate(root, name)

    candidate = telemetry.snapshot()['candidates'][0]
    comparison = candidate['comparison']

    assert comparison['same_frame'] is True
    assert comparison['same_resolution'] is True
    assert comparison['dimension_delta_cells'] == {
        'width': 0,
        'height': 0,
    }
    assert comparison['cell_count_delta'] == 0
    assert comparison['origin_delta_meters'] == {
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
    }


def test_candidate_endpoint_returns_inventory(
    monkeypatch,
):
    class FakeCandidateTelemetry:
        def snapshot(self):
            return {
                'available': True,
                'status': 'READY',
                'candidate_count': 2,
                'review_ready_count': 1,
                'invalid_count': 1,
                'read_only': True,
                'promotion_enabled': False,
                'candidates': [],
            }

    monkeypatch.setattr(
        bridge,
        'candidate_map_telemetry',
        FakeCandidateTelemetry(),
    )

    response = bridge.app.test_client().get(
        '/telemetry/map-candidates'
    )
    payload = response.get_json()

    assert response.status_code == 200
    assert payload['ok'] is True
    assert payload['source'] == 'candidate_map_review'
    assert payload['telemetry']['read_only'] is True
    assert payload['telemetry']['promotion_enabled'] is False
    assert response.headers[
        'Access-Control-Allow-Origin'
    ] == '*'


def test_candidate_endpoint_is_get_only():
    response = bridge.app.test_client().post(
        '/telemetry/map-candidates'
    )

    assert response.status_code == 405
