#!/usr/bin/env python3

#
# SPDX-License-Identifier: Apache-2.0
#

import hashlib
import json
import os
import re
import shutil
import threading
from pathlib import Path


class MapPromotionError(RuntimeError):
    pass


class MapPromotionConflictError(MapPromotionError):
    pass


class MapPromotion:
    CONFIRMATION = 'PROMOTE_REVIEWED_CANDIDATE'

    CANDIDATE_PATTERN = re.compile(
        r'^mayday_map_candidate_[0-9]{8}T'
        r'[0-9]{6}Z$'
    )

    VALIDATED_NAME = 'mayday_supervised_route_03'

    def __init__(
        self,
        map_root=None,
        candidate_map_telemetry=None,
        validated_map_telemetry=None,
        runtime_state_provider=None,
    ):
        home = Path.home()

        self._map_root = Path(
            map_root
            or home / 'robot_maps'
        )

        self._validated_directory = (
            self._map_root
            / self.VALIDATED_NAME
        )

        self._candidate_map_telemetry = (
            candidate_map_telemetry
        )
        self._validated_map_telemetry = (
            validated_map_telemetry
        )

        self._runtime_state_provider = (
            runtime_state_provider
            or (
                lambda: {
                    'mapping': {'running': False},
                    'localization': {'running': False},
                }
            )
        )

        self._lock = threading.RLock()
        self._last_promotion = None
        self._last_error = None

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()

        with Path(path).open('rb') as stream:
            while True:
                block = stream.read(1024 * 1024)

                if not block:
                    break

                digest.update(block)

        return digest.hexdigest()

    @classmethod
    def _timestamp_token(cls, timestamp):
        digits = ''.join(
            character
            for character in str(timestamp)
            if character.isdigit()
        )

        if len(digits) < 14:
            raise MapPromotionError(
                'Promotion timestamp is invalid.'
            )

        return (
            digits[:8]
            + 'T'
            + digits[8:14]
            + 'Z'
        )

    @classmethod
    def _canonical_names(cls):
        return (
            f'{cls.VALIDATED_NAME}.pbstream',
            f'{cls.VALIDATED_NAME}.yaml',
            f'{cls.VALIDATED_NAME}.pgm',
        )

    @classmethod
    def _candidate_names(cls, candidate_name):
        return (
            f'{candidate_name}.pbstream',
            f'{candidate_name}.yaml',
            f'{candidate_name}.pgm',
        )

    @classmethod
    def _write_manifest(cls, directory, names):
        directory = Path(directory)
        lines = []

        for name in names:
            path = directory / name

            if not path.is_file():
                raise MapPromotionError(
                    f'Missing promotion artifact: {name}'
                )

            lines.append(
                f'{cls._sha256(path)}  {name}'
            )

        (
            directory / 'SHA256SUMS'
        ).write_text(
            '\n'.join(lines) + '\n',
            encoding='utf-8',
        )

    @classmethod
    def _verify_manifest(cls, directory):
        directory = Path(directory)
        manifest = directory / 'SHA256SUMS'

        if not manifest.is_file():
            raise MapPromotionError(
                'SHA256SUMS is missing.'
            )

        entries = 0

        for raw_line in manifest.read_text(
            encoding='utf-8'
        ).splitlines():
            line = raw_line.strip()

            if not line:
                continue

            parts = line.split(None, 1)

            if len(parts) != 2:
                raise MapPromotionError(
                    'SHA256SUMS contains an invalid entry.'
                )

            expected = parts[0].strip()
            artifact_name = Path(
                parts[1].strip().lstrip('*')
            ).name
            artifact = directory / artifact_name

            if not artifact.is_file():
                raise MapPromotionError(
                    'Manifest artifact is missing: '
                    f'{artifact_name}'
                )

            if cls._sha256(artifact) != expected:
                raise MapPromotionError(
                    'Checksum mismatch: '
                    f'{artifact_name}'
                )

            entries += 1

        if entries < 3:
            raise MapPromotionError(
                'SHA256SUMS is incomplete.'
            )

    @classmethod
    def _normalize_yaml(
        cls,
        source_path,
        destination_path,
    ):
        lines = Path(source_path).read_text(
            encoding='utf-8'
        ).splitlines()

        output = []
        found_image = False

        for line in lines:
            if line.strip().startswith('image:'):
                output.append(
                    f'image: {cls.VALIDATED_NAME}.pgm'
                )
                found_image = True
            else:
                output.append(line)

        if not found_image:
            raise MapPromotionError(
                'Candidate YAML has no image entry.'
            )

        Path(destination_path).write_text(
            '\n'.join(output) + '\n',
            encoding='utf-8',
        )

    def _runtime_is_stopped(self):
        state = self._runtime_state_provider()

        mapping = state.get('mapping') or {}
        localization = state.get('localization') or {}

        return (
            not mapping.get('running')
            and not mapping.get('owned')
            and mapping.get('pid') is None
            and not localization.get('running')
            and not localization.get('owned')
            and localization.get('pid') is None
        )

    def _require_stopped_runtime(self):
        if not self._runtime_is_stopped():
            raise MapPromotionConflictError(
                'Mapping or localization is active; '
                'candidate promotion is blocked.'
            )

    def _review_candidate(self, candidate_name):
        if self._candidate_map_telemetry is None:
            raise MapPromotionError(
                'Candidate review telemetry is unavailable.'
            )

        telemetry = (
            self._candidate_map_telemetry.snapshot()
        )

        matches = [
            candidate
            for candidate in telemetry.get(
                'candidates',
                [],
            )
            if candidate.get('name') == candidate_name
        ]

        if len(matches) != 1:
            raise MapPromotionError(
                'Candidate was not uniquely identified.'
            )

        candidate = matches[0]

        if (
            candidate.get('classification')
            != 'REVIEW_READY'
            or candidate.get('review_ready') is not True
            or candidate.get('checksums_valid') is not True
            or candidate.get(
                'image_reference_valid'
            ) is not True
            or candidate.get('promoted') is not False
        ):
            raise MapPromotionError(
                'Candidate is not review-ready.'
            )

        comparison = candidate.get('comparison') or {}

        if (
            comparison.get('same_frame') is not True
            or comparison.get(
                'same_resolution'
            ) is not True
        ):
            raise MapPromotionError(
                'Candidate frame or resolution does not '
                'match the validated map.'
            )

        return candidate

    def _candidate_directory(
        self,
        candidate_name,
        review,
    ):
        expected = (
            self._map_root
            / candidate_name
        ).resolve()

        reported = Path(
            review.get('directory', expected)
        ).resolve()

        root = self._map_root.resolve()

        if (
            expected != reported
            or expected.parent != root
        ):
            raise MapPromotionError(
                'Candidate directory is outside the '
                'guarded map root.'
            )

        if not expected.is_dir():
            raise MapPromotionError(
                'Candidate directory does not exist.'
            )

        return expected

    def snapshot(self):
        with self._lock:
            return {
                'promotion_enabled': True,
                'confirmation_required': (
                    self.CONFIRMATION
                ),
                'validated_directory': str(
                    self._validated_directory
                ),
                'candidate_root': str(
                    self._map_root
                ),
                'last_promotion': self._last_promotion,
                'last_error': self._last_error,
                'mapping_required_stopped': True,
                'localization_required_stopped': True,
                'motion_enabled': False,
                'planning_enabled': False,
            }

    def promote(
        self,
        candidate_name,
        confirmation,
        timestamp,
    ):
        with self._lock:
            try:
                result = self._promote(
                    candidate_name,
                    confirmation,
                    timestamp,
                )
            except Exception as exc:
                self._last_error = str(exc)
                raise

            self._last_error = None
            self._last_promotion = result

            return result

    def _promote(
        self,
        candidate_name,
        confirmation,
        timestamp,
    ):
        if confirmation != self.CONFIRMATION:
            raise MapPromotionError(
                'Explicit promotion confirmation is '
                'required.'
            )

        if not isinstance(candidate_name, str):
            raise MapPromotionError(
                'Candidate name is invalid.'
            )

        if not self.CANDIDATE_PATTERN.fullmatch(
            candidate_name
        ):
            raise MapPromotionError(
                'Candidate name is invalid.'
            )

        self._require_stopped_runtime()

        review = self._review_candidate(
            candidate_name
        )
        candidate_directory = (
            self._candidate_directory(
                candidate_name,
                review,
            )
        )

        if not self._validated_directory.is_dir():
            raise MapPromotionError(
                'Validated map directory is missing.'
            )

        self._verify_manifest(
            candidate_directory
        )
        self._verify_manifest(
            self._validated_directory
        )

        token = self._timestamp_token(timestamp)

        backup_name = (
            'mayday_validated_backup_'
            + token
        )
        backup_directory = (
            self._map_root / backup_name
        )
        backup_partial = (
            self._map_root
            / f'.{backup_name}.partial'
        )
        staging_directory = (
            self._map_root
            / '.mayday_supervised_route_03.'
            f'promotion_{token}.partial'
        )
        previous_directory = (
            self._map_root
            / '.mayday_supervised_route_03.'
            f'previous_{token}.partial'
        )

        for path in (
            backup_directory,
            backup_partial,
            staging_directory,
            previous_directory,
        ):
            if path.exists():
                raise MapPromotionConflictError(
                    'Promotion target already exists: '
                    f'{path.name}'
                )

        candidate_names = self._candidate_names(
            candidate_name
        )
        canonical_names = self._canonical_names()

        try:
            shutil.copytree(
                self._validated_directory,
                backup_partial,
            )

            self._write_manifest(
                backup_partial,
                canonical_names,
            )
            self._verify_manifest(
                backup_partial
            )

            os.replace(
                backup_partial,
                backup_directory,
            )

            staging_directory.mkdir(
                mode=0o755
            )

            shutil.copy2(
                candidate_directory
                / candidate_names[0],
                staging_directory
                / canonical_names[0],
            )

            self._normalize_yaml(
                candidate_directory
                / candidate_names[1],
                staging_directory
                / canonical_names[1],
            )

            shutil.copy2(
                candidate_directory
                / candidate_names[2],
                staging_directory
                / canonical_names[2],
            )

            promotion_metadata = {
                'status': 'VALIDATED_MAP',
                'promoted': True,
                'promoted_at': timestamp,
                'source_candidate': candidate_name,
                'backup_directory': str(
                    backup_directory
                ),
                'validated_directory': str(
                    self._validated_directory
                ),
                'frame_id': 'map',
                'resolution': 0.05,
                'candidate_preserved': True,
                'mapping_enabled': False,
                'localization_enabled': False,
                'planning_enabled': False,
                'motion_enabled': False,
            }

            metadata_name = (
                'PROMOTION_METADATA.json'
            )

            (
                staging_directory / metadata_name
            ).write_text(
                json.dumps(
                    promotion_metadata,
                    indent=2,
                    sort_keys=True,
                ) + '\n',
                encoding='utf-8',
            )

            self._write_manifest(
                staging_directory,
                canonical_names
                + (metadata_name,),
            )
            self._verify_manifest(
                staging_directory
            )

            self._require_stopped_runtime()

            os.replace(
                self._validated_directory,
                previous_directory,
            )

            try:
                os.replace(
                    staging_directory,
                    self._validated_directory,
                )
            except Exception:
                os.replace(
                    previous_directory,
                    self._validated_directory,
                )
                raise

            self._verify_manifest(
                self._validated_directory
            )

            if self._validated_map_telemetry is not None:
                try:
                    self._validated_map_telemetry.reload()
                except Exception as error:
                    shutil.rmtree(
                        self._validated_directory
                    )
                    os.replace(
                        previous_directory,
                        self._validated_directory,
                    )

                    try:
                        self._validated_map_telemetry.reload()
                    except Exception as rollback_error:
                        raise MapPromotionError(
                            'Promoted-map telemetry reload failed '
                            'and rollback telemetry could not be '
                            'restored: '
                            f'{rollback_error}'
                        ) from error

                    raise MapPromotionError(
                        'Promoted map could not be loaded; '
                        'the validated map was rolled back: '
                        f'{error}'
                    ) from error

            shutil.rmtree(
                previous_directory
            )

        except Exception:
            if backup_partial.exists():
                shutil.rmtree(
                    backup_partial,
                    ignore_errors=True,
                )

            if staging_directory.exists():
                shutil.rmtree(
                    staging_directory,
                    ignore_errors=True,
                )

            if (
                previous_directory.exists()
                and not self._validated_directory.exists()
            ):
                os.replace(
                    previous_directory,
                    self._validated_directory,
                )

            raise

        return {
            'action': 'CANDIDATE_PROMOTED',
            'promoted': True,
            'candidate_name': candidate_name,
            'validated_directory': str(
                self._validated_directory
            ),
            'backup_directory': str(
                backup_directory
            ),
            'promotion_metadata': str(
                self._validated_directory
                / 'PROMOTION_METADATA.json'
            ),
            'candidate_preserved': True,
            'mapping_running': False,
            'localization_running': False,
            'planning_enabled': False,
            'motion_enabled': False,
            'timestamp': timestamp,
        }
