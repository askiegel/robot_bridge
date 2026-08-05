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

import ast
import hashlib
import threading
from datetime import datetime
from datetime import timezone
from pathlib import Path


DEFAULT_MAP_YAML = (
    Path.home()
    / 'robot_maps'
    / 'mayday_supervised_route_03'
    / 'mayday_supervised_route_03.yaml'
)


class SavedMapTelemetry:
    """Load one validated occupancy map as read-only HTTP telemetry."""

    def __init__(self, yaml_path=None, utc_clock=None):
        self._yaml_path = Path(
            yaml_path or DEFAULT_MAP_YAML
        ).expanduser()
        self._utc_clock = utc_clock or (
            lambda: datetime.now(timezone.utc)
        )
        self._lock = threading.Lock()
        self._map = None
        self._loaded_at = None
        self._error = None

    @staticmethod
    def _parse_scalar(value):
        value = value.strip()

        if value.startswith('['):
            return ast.literal_eval(value)

        if value in ('true', 'false'):
            return value == 'true'

        try:
            return int(value)
        except ValueError:
            pass

        try:
            return float(value)
        except ValueError:
            return value

    @classmethod
    def _parse_yaml(cls, yaml_path):
        metadata = {}

        for raw_line in yaml_path.read_text(
            encoding='utf-8',
        ).splitlines():
            line = raw_line.strip()

            if not line or line.startswith('#'):
                continue

            if ':' not in line:
                raise ValueError(
                    f'Invalid map metadata line: {raw_line}'
                )

            key, value = line.split(':', 1)
            metadata[key.strip()] = cls._parse_scalar(value)

        required = {
            'image',
            'mode',
            'resolution',
            'origin',
            'negate',
            'occupied_thresh',
            'free_thresh',
        }

        missing = sorted(required - metadata.keys())

        if missing:
            raise ValueError(
                f'Missing map metadata: {", ".join(missing)}'
            )

        origin = metadata['origin']

        if (
            not isinstance(origin, (list, tuple))
            or len(origin) != 3
        ):
            raise ValueError(
                'Map origin must contain x, y, and yaw.'
            )

        return metadata

    @staticmethod
    def _read_pgm_token(stream):
        token = bytearray()

        while True:
            byte = stream.read(1)

            if not byte:
                raise ValueError(
                    'Unexpected end of PGM header.'
                )

            if byte == b'#':
                stream.readline()
                continue

            if byte.isspace():
                continue

            token.extend(byte)
            break

        while True:
            byte = stream.read(1)

            if not byte or byte.isspace():
                break

            if byte == b'#':
                stream.readline()
                break

            token.extend(byte)

        return bytes(token)

    @classmethod
    def _read_pgm(cls, pgm_path):
        with pgm_path.open('rb') as stream:
            magic = cls._read_pgm_token(stream)
            width = int(cls._read_pgm_token(stream))
            height = int(cls._read_pgm_token(stream))
            maximum = int(cls._read_pgm_token(stream))
            pixels = stream.read()

        if magic != b'P5':
            raise ValueError('Only binary P5 PGM maps are supported.')

        if width <= 0 or height <= 0:
            raise ValueError('Map dimensions must be positive.')

        if maximum != 255:
            raise ValueError(
                'Only 8-bit PGM maps are supported.'
            )

        expected = width * height

        if len(pixels) != expected:
            raise ValueError(
                'PGM pixel count does not match its dimensions.'
            )

        return width, height, pixels

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()

        with path.open('rb') as source:
            while True:
                block = source.read(65536)

                if not block:
                    break

                digest.update(block)

        return digest.hexdigest()

    @staticmethod
    def _occupancy_value(
        pixel,
        negate,
        occupied_threshold,
        free_threshold,
    ):
        if negate:
            occupancy = float(pixel) / 255.0
        else:
            occupancy = float(255 - pixel) / 255.0

        if occupancy > occupied_threshold:
            return 100

        if occupancy < free_threshold:
            return 0

        return -1

    @classmethod
    def _convert_cells(
        cls,
        pixels,
        width,
        height,
        negate,
        occupied_threshold,
        free_threshold,
    ):
        cells = []

        # PGM rows begin at the upper-left. Occupancy-grid rows begin
        # at the map origin in the lower-left, so reverse row order.
        for source_y in range(height - 1, -1, -1):
            row_start = source_y * width
            row_end = row_start + width

            for pixel in pixels[row_start:row_end]:
                cells.append(
                    cls._occupancy_value(
                        pixel=pixel,
                        negate=negate,
                        occupied_threshold=occupied_threshold,
                        free_threshold=free_threshold,
                    )
                )

        return cells

    def _load(self):
        yaml_path = self._yaml_path.resolve()
        metadata = self._parse_yaml(yaml_path)

        image_path = Path(str(metadata['image']))

        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        image_path = image_path.resolve()

        if not image_path.is_file():
            raise FileNotFoundError(
                f'Map image not found: {image_path}'
            )

        width, height, pixels = self._read_pgm(image_path)

        resolution = float(metadata['resolution'])
        origin = [float(value) for value in metadata['origin']]
        negate = bool(int(metadata['negate']))
        occupied_threshold = float(
            metadata['occupied_thresh']
        )
        free_threshold = float(metadata['free_thresh'])

        if resolution <= 0.0:
            raise ValueError(
                'Map resolution must be positive.'
            )

        if not 0.0 <= free_threshold <= 1.0:
            raise ValueError(
                'Free threshold must be between zero and one.'
            )

        if not 0.0 <= occupied_threshold <= 1.0:
            raise ValueError(
                'Occupied threshold must be between zero and one.'
            )

        if free_threshold >= occupied_threshold:
            raise ValueError(
                'Free threshold must be below occupied threshold.'
            )

        cells = self._convert_cells(
            pixels=pixels,
            width=width,
            height=height,
            negate=negate,
            occupied_threshold=occupied_threshold,
            free_threshold=free_threshold,
        )

        unknown_count = cells.count(-1)
        free_count = cells.count(0)
        occupied_count = cells.count(100)

        return {
            'frame_id': 'map',
            'name': yaml_path.stem,
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin': {
                'x': origin[0],
                'y': origin[1],
                'yaw': origin[2],
            },
            'cell_count': len(cells),
            'unknown_cell_count': unknown_count,
            'free_cell_count': free_count,
            'occupied_cell_count': occupied_count,
            'encoding': 'ros_occupancy_values',
            'unknown_value': -1,
            'free_value': 0,
            'occupied_value': 100,
            'cells': cells,
            'source': {
                'yaml_name': yaml_path.name,
                'image_name': image_path.name,
                'yaml_sha256': self._sha256(yaml_path),
                'image_sha256': self._sha256(image_path),
            },
        }

    def snapshot(self):
        """Return a copy of the cached map or an unavailable result."""
        with self._lock:
            if self._map is None and self._error is None:
                try:
                    self._map = self._load()
                    self._loaded_at = self._utc_clock().isoformat()
                except Exception as error:
                    self._error = str(error)

            if self._map is None:
                return {
                    'available': False,
                    'status': 'MAP_UNAVAILABLE',
                    'loaded_at': None,
                    'error': self._error,
                    'map': None,
                }

            occupancy_map = dict(self._map)
            occupancy_map['origin'] = dict(
                self._map['origin']
            )
            occupancy_map['source'] = dict(
                self._map['source']
            )
            occupancy_map['cells'] = list(
                self._map['cells']
            )

            return {
                'available': True,
                'status': 'READY',
                'loaded_at': self._loaded_at,
                'error': None,
                'map': occupancy_map,
            }
