#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import hashlib
import json
import re
from datetime import datetime
from datetime import timezone
from pathlib import Path

from map_telemetry import SavedMapTelemetry


DEFAULT_MAP_ROOT = Path.home() / 'robot_maps'
CANDIDATE_PATTERN = re.compile(
    r'^mayday_map_candidate_\d{8}T\d{6}Z$'
)


class CandidateMapTelemetry:
    """Inspect candidate maps without changing or promoting them."""

    def __init__(
        self,
        map_root=None,
        validated_map_telemetry=None,
        utc_clock=None,
    ):
        self._map_root = Path(
            map_root or DEFAULT_MAP_ROOT
        ).expanduser()
        self._validated_map_telemetry = (
            validated_map_telemetry
            or SavedMapTelemetry()
        )
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()

        with path.open('rb') as stream:
            while True:
                block = stream.read(65536)

                if not block:
                    break

                digest.update(block)

        return digest.hexdigest()

    @staticmethod
    def _read_manifest(path):
        entries = {}

        for raw_line in path.read_text(
            encoding='utf-8',
        ).splitlines():
            fields = raw_line.split()

            if len(fields) != 2:
                continue

            digest, filename = fields
            entries[Path(filename.lstrip('*')).name] = (
                digest
            )

        return entries

    @staticmethod
    def _map_summary(occupancy_map):
        return {
            key: occupancy_map[key]
            for key in (
                'frame_id',
                'name',
                'width',
                'height',
                'resolution',
                'origin',
                'cell_count',
                'unknown_cell_count',
                'free_cell_count',
                'occupied_cell_count',
                'encoding',
                'unknown_value',
                'free_value',
                'occupied_value',
                'source',
            )
        }

    @staticmethod
    def _bounds(occupancy_map):
        origin = occupancy_map['origin']
        width_meters = (
            occupancy_map['width']
            * occupancy_map['resolution']
        )
        height_meters = (
            occupancy_map['height']
            * occupancy_map['resolution']
        )

        return {
            'minimum_x': origin['x'],
            'minimum_y': origin['y'],
            'maximum_x': origin['x'] + width_meters,
            'maximum_y': origin['y'] + height_meters,
            'width_meters': width_meters,
            'height_meters': height_meters,
        }

    @classmethod
    def _comparison(cls, validated, candidate):
        validated_bounds = cls._bounds(validated)
        candidate_bounds = cls._bounds(candidate)

        return {
            'same_frame': (
                validated['frame_id']
                == candidate['frame_id']
            ),
            'same_resolution': (
                abs(
                    validated['resolution']
                    - candidate['resolution']
                )
                < 1e-9
            ),
            'dimension_delta_cells': {
                'width': (
                    candidate['width']
                    - validated['width']
                ),
                'height': (
                    candidate['height']
                    - validated['height']
                ),
            },
            'origin_delta_meters': {
                'x': (
                    candidate['origin']['x']
                    - validated['origin']['x']
                ),
                'y': (
                    candidate['origin']['y']
                    - validated['origin']['y']
                ),
                'yaw': (
                    candidate['origin']['yaw']
                    - validated['origin']['yaw']
                ),
            },
            'cell_count_delta': (
                candidate['cell_count']
                - validated['cell_count']
            ),
            'unknown_cell_delta': (
                candidate['unknown_cell_count']
                - validated['unknown_cell_count']
            ),
            'free_cell_delta': (
                candidate['free_cell_count']
                - validated['free_cell_count']
            ),
            'occupied_cell_delta': (
                candidate['occupied_cell_count']
                - validated['occupied_cell_count']
            ),
            'validated_bounds': validated_bounds,
            'candidate_bounds': candidate_bounds,
            'overlap_bounds': {
                'minimum_x': max(
                    validated_bounds['minimum_x'],
                    candidate_bounds['minimum_x'],
                ),
                'minimum_y': max(
                    validated_bounds['minimum_y'],
                    candidate_bounds['minimum_y'],
                ),
                'maximum_x': min(
                    validated_bounds['maximum_x'],
                    candidate_bounds['maximum_x'],
                ),
                'maximum_y': min(
                    validated_bounds['maximum_y'],
                    candidate_bounds['maximum_y'],
                ),
            },
        }

    def _inspect_candidate(
        self,
        directory,
        validated_map,
    ):
        name = directory.name
        reasons = []

        if CANDIDATE_PATTERN.fullmatch(name) is None:
            reasons.append('INVALID_CANDIDATE_NAME')

        paths = {
            'pbstream': directory / f'{name}.pbstream',
            'yaml': directory / f'{name}.yaml',
            'image': directory / f'{name}.pgm',
            'metadata': (
                directory / 'CANDIDATE_METADATA.json'
            ),
            'checksums': directory / 'SHA256SUMS',
        }

        missing = [
            path.name
            for path in paths.values()
            if not path.is_file()
        ]

        if missing:
            return {
                'name': name,
                'directory': str(directory),
                'classification': 'INVALID_MISSING_FILES',
                'review_ready': False,
                'promoted': False,
                'reasons': [
                    f'MISSING:{filename}'
                    for filename in missing
                ],
                'metadata': None,
                'map': None,
                'comparison': None,
                'checksums_valid': False,
                'image_reference_valid': False,
            }

        try:
            manifest = self._read_manifest(
                paths['checksums']
            )
        except Exception as error:
            reasons.append(
                f'INVALID_CHECKSUM_MANIFEST:{error}'
            )
            manifest = {}

        checksum_errors = []

        for key in (
            'pbstream',
            'yaml',
            'image',
            'metadata',
        ):
            artifact = paths[key]
            expected = manifest.get(artifact.name)

            try:
                actual = self._sha256(artifact)
            except Exception as error:
                checksum_errors.append(
                    f'{artifact.name}:{error}'
                )
                continue

            if expected != actual:
                checksum_errors.append(artifact.name)

        try:
            metadata = json.loads(
                paths['metadata'].read_text(
                    encoding='utf-8',
                )
            )
        except Exception as error:
            metadata = None
            reasons.append(f'INVALID_METADATA:{error}')

        metadata_valid = (
            isinstance(metadata, dict)
            and metadata.get('candidate_name') == name
            and metadata.get('status')
            == 'CANDIDATE_REVIEW_REQUIRED'
            and metadata.get('promoted') is False
            and metadata.get(
                'validated_map_changed'
            ) is False
        )

        if not metadata_valid:
            reasons.append('INVALID_METADATA')

        try:
            yaml_metadata = SavedMapTelemetry._parse_yaml(
                paths['yaml']
            )
            image_value = str(yaml_metadata['image'])
        except Exception as error:
            image_value = None
            reasons.append(f'INVALID_YAML:{error}')

        image_reference_valid = False

        if image_value is not None:
            image_reference = Path(image_value)

            image_reference_valid = (
                not image_reference.is_absolute()
                and image_reference.name
                == paths['image'].name
                and image_reference.parent
                == Path('.')
                and (
                    directory / image_reference
                ).resolve()
                == paths['image'].resolve()
                and paths['image'].is_file()
            )

        if not image_reference_valid:
            reasons.append('INVALID_IMAGE_REFERENCE')

        if checksum_errors:
            reasons.append('INVALID_CHECKSUM')

        if checksum_errors:
            classification = 'INVALID_CHECKSUM'
        elif not metadata_valid:
            classification = 'INVALID_METADATA'
        elif not image_reference_valid:
            classification = 'INVALID_IMAGE_REFERENCE'
        elif reasons:
            classification = reasons[0].split(':', 1)[0]
        else:
            classification = 'REVIEW_READY'

        occupancy_map = None
        comparison = None

        if classification == 'REVIEW_READY':
            map_snapshot = SavedMapTelemetry(
                yaml_path=paths['yaml'],
                utc_clock=self._utc_clock,
            ).snapshot()

            if not map_snapshot['available']:
                classification = 'INVALID_MAP'
                reasons.append(
                    'INVALID_MAP:'
                    + str(map_snapshot['error'])
                )
            else:
                occupancy_map = map_snapshot['map']
                comparison = self._comparison(
                    validated_map,
                    occupancy_map,
                )

        return {
            'name': name,
            'directory': str(directory),
            'classification': classification,
            'review_ready': (
                classification == 'REVIEW_READY'
            ),
            'promoted': (
                metadata.get('promoted')
                if isinstance(metadata, dict)
                else False
            ),
            'reasons': reasons,
            'metadata': metadata,
            'map': occupancy_map,
            'map_summary': (
                self._map_summary(occupancy_map)
                if occupancy_map is not None
                else None
            ),
            'comparison': comparison,
            'checksums_valid': not checksum_errors,
            'checksum_errors': checksum_errors,
            'image_reference': image_value,
            'image_reference_valid': (
                image_reference_valid
            ),
            'artifacts': {
                key: str(value)
                for key, value in paths.items()
            },
        }

    def snapshot(self):
        """Return a fresh read-only candidate inventory."""
        validated_snapshot = (
            self._validated_map_telemetry.snapshot()
        )

        if not validated_snapshot['available']:
            return {
                'available': False,
                'status': 'VALIDATED_MAP_UNAVAILABLE',
                'inspected_at': (
                    self._utc_clock().isoformat()
                ),
                'error': validated_snapshot['error'],
                'validated_map': None,
                'candidate_count': 0,
                'review_ready_count': 0,
                'invalid_count': 0,
                'candidates': [],
            }

        validated_map = validated_snapshot['map']
        candidates = []

        if self._map_root.is_dir():
            directories = sorted(
                path
                for path in self._map_root.iterdir()
                if (
                    path.is_dir()
                    and path.name.startswith(
                        'mayday_map_candidate_'
                    )
                )
            )
        else:
            directories = []

        for directory in directories:
            candidates.append(
                self._inspect_candidate(
                    directory,
                    validated_map,
                )
            )

        review_ready_count = sum(
            candidate['review_ready']
            for candidate in candidates
        )

        return {
            'available': True,
            'status': 'READY',
            'inspected_at': self._utc_clock().isoformat(),
            'error': None,
            'validated_map': self._map_summary(
                validated_map
            ),
            'candidate_count': len(candidates),
            'review_ready_count': review_ready_count,
            'invalid_count': (
                len(candidates) - review_ready_count
            ),
            'candidates': candidates,
            'read_only': True,
            'promotion_enabled': False,
        }
