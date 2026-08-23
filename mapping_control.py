#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import threading
from pathlib import Path


class MappingControlError(RuntimeError):
    """A mapping session could not be controlled safely."""


class MappingConflictError(MappingControlError):
    """Another mapping, localization, or Nav2 process exists."""


class MappingControl:
    """Own one headless, mapping-only Cartographer session."""

    ROS2_EXECUTABLE = Path('/opt/ros/humble/bin/ros2')
    PACKAGE = 'mini_pupper_slam'
    LAUNCH_FILE = 'slam.launch.py'
    SERVICE = 'mayday-slam.service'
    SERVICE_CGROUP = '/system.slice/mayday-slam.service'
    SYSTEMCTL_EXECUTABLE = Path('/usr/bin/systemctl')
    SUDO_EXECUTABLE = Path('/usr/bin/sudo')

    def __init__(
        self,
        launch_path=None,
        log_path=None,
        candidate_root=None,
        converter_path=None,
        command_runner=None,
        service_runner=None,
        process_factory=None,
        process_finder=None,
        identity_checker=None,
        get_process_group=None,
        signal_process_group=None,
    ):
        home = Path.home()

        self._launch_path = Path(
            launch_path
            or home
            / 'ros2_ws'
            / 'install'
            / self.PACKAGE
            / 'share'
            / self.PACKAGE
            / 'launch'
            / self.LAUNCH_FILE
        )
        self._log_path = Path(
            log_path
            or home
            / 'robot_bridge'
            / 'logs'
            / 'mapping_control.log'
        )
        self._candidate_root = Path(
            candidate_root
            or home / 'robot_maps'
        )
        self._converter_path = Path(
            converter_path
            or '/opt/ros/humble/lib/cartographer_ros/'
            'cartographer_pbstream_to_ros_map'
        )
        self._command_runner = (
            command_runner or subprocess.run
        )
        self._service_runner = (
            service_runner or subprocess.run
        )

        self._process_factory = (
            process_factory or subprocess.Popen
        )
        self._process_finder = (
            process_finder
            or self._find_external_processes
        )
        self._identity_checker = (
            identity_checker
            or self._verify_process_identity
        )
        self._get_process_group = (
            get_process_group or os.getpgid
        )
        self._signal_process_group = (
            signal_process_group or os.killpg
        )

        self._lock = threading.RLock()
        self._process = None
        self._started_at = None
        self._last_stopped_at = None
        self._last_error = None
        self._last_candidate = None

    @property
    def command(self):
        """Return the fixed headless mapping-only command."""
        return [
            str(self.ROS2_EXECUTABLE),
            'launch',
            self.PACKAGE,
            self.LAUNCH_FILE,
            'use_sim_time:=false',
            'use_rviz:=false',
        ]

    def _query_service(self):
        """Return authoritative systemd state for mapping."""
        command = [
            str(self.SYSTEMCTL_EXECUTABLE),
            'show',
            self.SERVICE,
            '-p',
            'ActiveState',
            '-p',
            'SubState',
            '-p',
            'MainPID',
            '-p',
            'ControlGroup',
            '--no-pager',
        ]

        try:
            completed = self._service_runner(
                command,
                cwd=str(Path.home()),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=10.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise MappingControlError(
                'Mapping systemd status query timed out.'
            ) from exc
        except OSError as exc:
            raise MappingControlError(
                f'Mapping systemd status query failed: {exc}'
            ) from exc

        output = str(
            getattr(completed, 'stdout', '')
        ).strip()

        if completed.returncode != 0:
            raise MappingControlError(
                'Mapping systemd status query failed'
                + (f': {output}' if output else '.')
            )

        properties = {}

        for line in output.splitlines():
            if '=' not in line:
                continue

            key, value = line.split('=', 1)
            properties[key.strip()] = value.strip()

        try:
            pid = int(
                properties.get('MainPID', '0') or '0'
            )
        except ValueError as exc:
            raise MappingControlError(
                'Mapping systemd MainPID is invalid.'
            ) from exc

        active_state = properties.get(
            'ActiveState',
            'unknown',
        )
        sub_state = properties.get(
            'SubState',
            'unknown',
        )
        control_group = properties.get(
            'ControlGroup',
            '',
        )

        running = bool(
            active_state == 'active'
            and sub_state == 'running'
            and pid > 0
        )
        owned = bool(
            running
            and control_group == self.SERVICE_CGROUP
        )

        return {
            'active_state': active_state,
            'sub_state': sub_state,
            'running': running,
            'owned': owned,
            'pid': pid if pid > 0 else None,
            'control_group': control_group,
        }

    def _run_systemctl(self, action):
        """Run one fixed privileged mapping service operation."""
        if action not in {'start', 'stop'}:
            raise MappingControlError(
                'Unsupported mapping systemd operation.'
            )

        command = [
            str(self.SUDO_EXECUTABLE),
            '-n',
            str(self.SYSTEMCTL_EXECUTABLE),
            action,
            self.SERVICE,
        ]

        try:
            completed = self._service_runner(
                command,
                cwd=str(Path.home()),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise MappingControlError(
                f'Mapping systemd {action} timed out.'
            ) from exc
        except OSError as exc:
            raise MappingControlError(
                f'Mapping systemd {action} failed: {exc}'
            ) from exc

        output = str(
            getattr(completed, 'stdout', '')
        ).strip()

        if completed.returncode != 0:
            raise MappingControlError(
                f'Mapping systemd {action} failed'
                + (f': {output}' if output else '.')
            )

    def snapshot(self):
        """Return mapping ownership without changing runtime."""
        with self._lock:
            service_error = None

            try:
                service = self._query_service()
            except MappingControlError as exc:
                service_error = str(exc)
                service = {
                    'active_state': 'unknown',
                    'sub_state': 'unknown',
                    'running': False,
                    'owned': False,
                    'pid': None,
                    'control_group': '',
                }

            running = service['running']
            owned = service['owned']

            if running and owned:
                state = 'RUNNING'
            elif service['active_state'] == 'active':
                state = 'SERVICE_CONFLICT'
            elif service_error is not None:
                state = 'UNAVAILABLE'
            else:
                state = 'STOPPED'

            return {
                'running': running,
                'owned': owned,
                'pid': service['pid'],
                'state': state,
                'started_at': self._started_at,
                'last_stopped_at': self._last_stopped_at,
                'last_error': (
                    service_error or self._last_error
                ),
                'systemd_service': self.SERVICE,
                'service_active_state': (
                    service['active_state']
                ),
                'service_sub_state': service['sub_state'],
                'control_group': service['control_group'],
                'launch': (
                    f'{self.PACKAGE}/'
                    f'{self.LAUNCH_FILE}'
                ),
                'headless': True,
                'planning_enabled': False,
                'control_enabled': False,
                'map_save_enabled': True,
                'validated_map_mutable': False,
                'candidate_minimum_submaps': 3,
                'candidate_minimum_mature_submaps': 2,
                'candidate_minimum_mature_version': 100,
                'candidate_root': str(
                    self._candidate_root
                ),
                'last_candidate': self._last_candidate,
            }

    def start(self, timestamp):
        """Start the protected systemd-owned mapping service."""
        with self._lock:
            current = self._query_service()

            if current['running']:
                if not current['owned']:
                    raise MappingConflictError(
                        'Mapping service is active outside '
                        'the expected systemd cgroup.'
                    )

                result = self.snapshot()
                result['action'] = 'ALREADY_RUNNING'
                result['started'] = False
                return result

            if current['active_state'] == 'active':
                raise MappingConflictError(
                    'Mapping service is active but is not '
                    'a valid owned runtime.'
                )

            external = self._process_finder(None)

            if external:
                raise MappingConflictError(
                    'Mapping, localization, or Nav2 processes '
                    'already exist outside systemd ownership: '
                    + ', '.join(
                        str(pid)
                        for pid in external
                    )
                )

            self._verify_artifacts()

            self._run_systemctl('start')

            started = self._query_service()

            if not (
                started['running']
                and started['owned']
            ):
                self._last_error = (
                    'mayday-slam.service did not enter the '
                    'expected active systemd cgroup.'
                )
                raise MappingControlError(
                    self._last_error
                )

            self._started_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result['action'] = 'STARTED'
            result['started'] = True
            return result

    def save_candidate(self, timestamp):
        """
        Finish and export the owned session as a candidate.

        The validated map is never read, replaced, promoted,
        renamed, or removed by this method.
        """
        with self._lock:
            runtime = self._query_service()

            if not (
                runtime['running']
                and runtime['owned']
            ):
                raise MappingControlError(
                    'No owned mapping session is running.'
                )

            pid = runtime['pid']

            self._verify_export_tools()

            candidate_name = self._candidate_name(
                timestamp
            )
            final_directory = (
                self._candidate_root / candidate_name
            )
            temporary_directory = (
                self._candidate_root
                / f'.{candidate_name}.partial'
            )

            if (
                final_directory.exists()
                or temporary_directory.exists()
            ):
                raise MappingControlError(
                    'Candidate output directory already exists.'
                )

            temporary_directory.mkdir(
                parents=False,
                exist_ok=False,
            )

            filestem = (
                temporary_directory / candidate_name
            )
            pbstream_path = filestem.with_suffix(
                '.pbstream'
            )

            try:
                trajectory_id = (
                    self._get_active_trajectory_id()
                )
                readiness = (
                    self._get_submap_readiness(
                        trajectory_id,
                    )
                )

                self._run_service(
                    [
                        str(self.ROS2_EXECUTABLE),
                        'service',
                        'call',
                        '/finish_trajectory',
                        'cartographer_ros_msgs/srv/'
                        'FinishTrajectory',
                        (
                            '{trajectory_id: '
                            f'{trajectory_id}'
                            '}'
                        ),
                    ],
                    'FinishTrajectory',
                )

                self._run_service(
                    [
                        str(self.ROS2_EXECUTABLE),
                        'service',
                        'call',
                        '/write_state',
                        'cartographer_ros_msgs/srv/'
                        'WriteState',
                        (
                            "{filename: '"
                            f"{pbstream_path}"
                            "', include_unfinished_submaps: "
                            'false}'
                        ),
                    ],
                    'WriteState',
                )

                if (
                    not pbstream_path.is_file()
                    or pbstream_path.stat().st_size == 0
                ):
                    raise MappingControlError(
                        'WriteState did not create a PBStream.'
                    )

                stop_result = self.stop(timestamp)

                self._run_checked(
                    [
                        str(self._converter_path),
                        (
                            '-pbstream_filename='
                            f'{pbstream_path}'
                        ),
                        f'-map_filestem={filestem}',
                        '-resolution=0.05',
                    ],
                    'PBStream conversion',
                    timeout=90.0,
                )

                yaml_path = filestem.with_suffix('.yaml')
                image_path = filestem.with_suffix('.pgm')

                required = (
                    pbstream_path,
                    yaml_path,
                    image_path,
                )

                for artifact in required:
                    if (
                        not artifact.is_file()
                        or artifact.stat().st_size == 0
                    ):
                        raise MappingControlError(
                            'Candidate artifact is missing: '
                            f'{artifact.name}'
                        )

                yaml_source = yaml_path.read_text(
                    encoding='utf-8',
                )
                normalized_yaml, replacements = (
                    re.subn(
                        r'(?m)^image:\s*.+?\s*$',
                        f'image: {image_path.name}',
                        yaml_source,
                        count=1,
                    )
                )

                if replacements != 1:
                    raise MappingControlError(
                        'Candidate YAML image field '
                        'could not be normalized.'
                    )

                yaml_path.write_text(
                    normalized_yaml.rstrip() + '\n',
                    encoding='utf-8',
                )

                resolved_image = (
                    yaml_path.parent / image_path.name
                )

                if not resolved_image.is_file():
                    raise MappingControlError(
                        'Normalized candidate image '
                        'reference does not resolve.'
                    )

                metadata = {
                    'status': 'CANDIDATE_REVIEW_REQUIRED',
                    'promoted': False,
                    'validated_map_changed': False,
                    'candidate_name': candidate_name,
                    'created_at': timestamp,
                    'trajectory_id': trajectory_id,
                    'submap_readiness': readiness,
                    'resolution': 0.05,
                    'frame_id': 'map',
                    'artifacts': [
                        artifact.name
                        for artifact in required
                    ],
                }

                metadata_path = (
                    temporary_directory
                    / 'CANDIDATE_METADATA.json'
                )
                metadata_path.write_text(
                    json.dumps(
                        metadata,
                        indent=2,
                        sort_keys=True,
                    )
                    + '\n',
                    encoding='utf-8',
                )

                checksum_paths = (
                    *required,
                    metadata_path,
                )
                checksum_lines = []

                for artifact in checksum_paths:
                    digest = hashlib.sha256(
                        artifact.read_bytes()
                    ).hexdigest()
                    checksum_lines.append(
                        f'{digest}  {artifact.name}'
                    )

                checksum_path = (
                    temporary_directory / 'SHA256SUMS'
                )
                checksum_path.write_text(
                    '\n'.join(checksum_lines) + '\n',
                    encoding='utf-8',
                )

                temporary_directory.rename(
                    final_directory
                )

                self._last_candidate = {
                    'name': candidate_name,
                    'directory': str(final_directory),
                    'pbstream': str(
                        final_directory
                        / pbstream_path.name
                    ),
                    'yaml': str(
                        final_directory
                        / yaml_path.name
                    ),
                    'image': str(
                        final_directory
                        / image_path.name
                    ),
                    'metadata': str(
                        final_directory
                        / metadata_path.name
                    ),
                    'checksums': str(
                        final_directory
                        / checksum_path.name
                    ),
                    'status': (
                        'CANDIDATE_REVIEW_REQUIRED'
                    ),
                    'promoted': False,
                }
                self._last_error = None

                result = self.snapshot()
                result['action'] = 'CANDIDATE_SAVED'
                result['saved'] = True
                result['candidate'] = (
                    self._last_candidate
                )
                result['stopped_pid'] = (
                    stop_result.get('stopped_pid')
                )
                return result

            except Exception as exc:
                try:
                    runtime = self._query_service()

                    if (
                        runtime['running']
                        and runtime['owned']
                    ):
                        self.stop(timestamp)
                except Exception:
                    pass

                if temporary_directory.exists():
                    resolved_temporary = (
                        temporary_directory.resolve()
                    )
                    resolved_root = (
                        self._candidate_root.resolve()
                    )

                    if (
                        resolved_temporary.parent
                        == resolved_root
                        and resolved_temporary.name.startswith(
                            '.mayday_map_candidate_'
                        )
                        and resolved_temporary.name.endswith(
                            '.partial'
                        )
                    ):
                        shutil.rmtree(
                            resolved_temporary
                        )

                self._last_error = str(exc)

                if isinstance(exc, MappingControlError):
                    raise

                raise MappingControlError(
                    f'Candidate export failed: {exc}'
                ) from exc

    def stop(self, timestamp):
        """Stop only the protected systemd mapping service."""
        with self._lock:
            current = self._query_service()

            if current['active_state'] != 'active':
                result = self.snapshot()
                result['action'] = 'ALREADY_STOPPED'
                result['stopped'] = False
                return result

            if not (
                current['running']
                and current['owned']
            ):
                self._last_error = (
                    'Mapping service is active outside '
                    'the expected systemd cgroup; '
                    'stop was refused.'
                )
                raise MappingControlError(
                    self._last_error
                )

            pid = current['pid']

            self._run_systemctl('stop')

            stopped = self._query_service()

            if (
                stopped['active_state'] == 'active'
                or stopped['running']
                or stopped['pid'] is not None
            ):
                self._last_error = (
                    'mayday-slam.service remained active '
                    'after systemd stop.'
                )
                raise MappingControlError(
                    self._last_error
                )

            self._started_at = None
            self._last_stopped_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result['action'] = 'STOPPED'
            result['stopped'] = True
            result['stopped_pid'] = pid
            return result

    def shutdown(self):
        """
        Leave systemd-owned mapping untouched when Robot Bridge exits.

        Mapping lifetime is independent of the Robot Bridge process.
        Explicit /mapping/stop remains the supported stop operation.
        """
        return None

    @staticmethod
    def _candidate_name(timestamp):
        match = re.match(
            r'^(\d{4})-(\d{2})-(\d{2})'
            r'T(\d{2}):(\d{2}):(\d{2})',
            str(timestamp),
        )

        if match is None:
            raise MappingControlError(
                'Candidate timestamp is invalid.'
            )

        fields = match.groups()
        compact_date = ''.join(fields[:3])
        compact_time = ''.join(fields[3:])

        return (
            'mayday_map_candidate_'
            f'{compact_date}T{compact_time}Z'
        )

    def _verify_export_tools(self):
        if (
            not self._candidate_root.is_dir()
            or not os.access(
                self._candidate_root,
                os.W_OK,
            )
        ):
            raise MappingControlError(
                'Candidate map root is unavailable.'
            )

        if (
            not self._converter_path.is_file()
            or not os.access(
                self._converter_path,
                os.X_OK,
            )
        ):
            raise MappingControlError(
                'PBStream converter is unavailable.'
            )

    def _run_checked(
        self,
        command,
        operation,
        timeout,
    ):
        try:
            completed = self._command_runner(
                command,
                cwd=str(Path.home()),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise MappingControlError(
                f'{operation} timed out.'
            ) from exc

        if completed.returncode != 0:
            output = str(
                getattr(completed, 'stdout', '')
            ).strip()

            raise MappingControlError(
                f'{operation} failed'
                + (f': {output}' if output else '.')
            )

        return str(
            getattr(completed, 'stdout', '')
        )

    def _run_service(self, command, operation):
        output = self._run_checked(
            command,
            operation,
            timeout=45.0,
        )

        if not re.search(r'code=0\b', output):
            raise MappingControlError(
                f'{operation} returned an error: '
                f'{output.strip()}'
            )

        return output

    def _get_submap_readiness(
        self,
        trajectory_id,
    ):
        output = self._run_checked(
            [
                str(self.ROS2_EXECUTABLE),
                'topic',
                'echo',
                '/submap_list',
                'cartographer_ros_msgs/msg/SubmapList',
                '--once',
            ],
            'Submap readiness check',
            timeout=45.0,
        )

        blocks = re.findall(
            r'(?ms)^- trajectory_id:\s*(-?\d+)'
            r'.*?^\s+submap_index:\s*(\d+)'
            r'.*?^\s+submap_version:\s*(\d+)',
            output,
        )

        matching = [
            {
                'index': int(index),
                'version': int(version),
            }
            for found_trajectory, index, version
            in blocks
            if int(found_trajectory) == trajectory_id
        ]

        mature = [
            submap
            for submap in matching
            if submap['version'] >= 100
        ]

        readiness = {
            'trajectory_id': trajectory_id,
            'submap_count': len(matching),
            'mature_submap_count': len(mature),
            'minimum_submap_count': 3,
            'minimum_mature_submap_count': 2,
            'minimum_mature_version': 100,
            'submaps': matching,
        }

        if (
            len(matching) < 3
            or len(mature) < 2
        ):
            raise MappingControlError(
                'Mapping is not ready for candidate '
                'export: '
                f'{len(matching)} submaps, '
                f'{len(mature)} mature submaps; '
                'at least 3 submaps and 2 mature '
                'submaps are required.'
            )

        return readiness

    def _get_active_trajectory_id(self):
        output = self._run_service(
            [
                str(self.ROS2_EXECUTABLE),
                'service',
                'call',
                '/get_trajectory_states',
                'cartographer_ros_msgs/srv/'
                'GetTrajectoryStates',
                '{}',
            ],
            'GetTrajectoryStates',
        )

        identifiers = re.findall(
            r'trajectory_id=\[([^\]]*)\]',
            output,
        )
        states = re.findall(
            r'trajectory_state=\[([^\]]*)\]',
            output,
        )

        if not identifiers or not states:
            raise MappingControlError(
                'Trajectory state response was invalid.'
            )

        trajectory_ids = [
            int(value)
            for value in re.findall(
                r'-?\d+',
                identifiers[-1],
            )
        ]
        trajectory_states = [
            int(value)
            for value in re.findall(
                r'\d+',
                states[-1],
            )
        ]

        active = [
            trajectory_id
            for trajectory_id, state
            in zip(
                trajectory_ids,
                trajectory_states,
            )
            if state == 0
        ]

        if len(active) != 1:
            raise MappingControlError(
                'Exactly one active trajectory is required.'
            )

        return active[0]

    def _verify_artifacts(self):
        if not self.ROS2_EXECUTABLE.is_file():
            raise MappingControlError(
                'ROS 2 executable is unavailable.'
            )

        if not self._launch_path.is_file():
            raise MappingControlError(
                'Installed headless SLAM launch is unavailable.'
            )

    @classmethod
    def _is_conflicting_command(cls, command):
        normalized = ' '.join(
            str(command).split()
        )

        launch_markers = (
            cls.PACKAGE,
            cls.LAUNCH_FILE,
        )

        if all(
            marker in normalized
            for marker in launch_markers
        ):
            return True

        conflicting_markers = (
            '/cartographer_ros/cartographer_node',
            '/cartographer_ros/'
            'cartographer_occupancy_grid_node',
            '/nav2_amcl/amcl',
            '/nav2_map_server/map_server',
            '/nav2_lifecycle_manager/lifecycle_manager',
            '/nav2_planner/planner_server',
            '/nav2_controller/controller_server',
            '/nav2_bt_navigator/bt_navigator',
            '/nav2_behaviors/behavior_server',
            '/nav2_velocity_smoother/velocity_smoother',
            'mini_pupper_navigation '
            'localization.launch.py',
            'mini_pupper_navigation '
            'planning.launch.py',
        )

        return any(
            marker in normalized
            for marker in conflicting_markers
        )

    @classmethod
    def _find_external_processes(cls, owned_pid):
        found = []

        for process_directory in Path('/proc').iterdir():
            if not process_directory.name.isdigit():
                continue

            pid = int(process_directory.name)

            if owned_pid is not None and pid == owned_pid:
                continue

            try:
                command = (
                    process_directory
                    / 'cmdline'
                ).read_bytes().replace(
                    b'\0',
                    b' ',
                ).decode(
                    'utf-8',
                    errors='replace',
                )
            except (OSError, PermissionError):
                continue

            if cls._is_conflicting_command(command):
                found.append(pid)

        return sorted(found)

    @classmethod
    def _verify_process_identity(cls, pid):
        try:
            command = (
                Path('/proc')
                / str(int(pid))
                / 'cmdline'
            ).read_bytes().replace(
                b'\0',
                b' ',
            ).decode(
                'utf-8',
                errors='replace',
            )
        except (OSError, PermissionError, ValueError):
            return False

        return (
            'ros2' in command
            and 'launch' in command
            and cls.PACKAGE in command
            and cls.LAUNCH_FILE in command
        )
