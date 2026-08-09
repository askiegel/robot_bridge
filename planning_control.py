#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import hashlib
import os
import signal
import subprocess
import threading
from pathlib import Path


class PlanningControlError(RuntimeError):
    """Planning could not be controlled safely."""


class PlanningConflictError(PlanningControlError):
    """Another process already owns planning."""


class PlanningControl:
    """Own one conservative planning-only launch session."""

    ROS2_EXECUTABLE = Path('/opt/ros/humble/bin/ros2')
    PACKAGE = 'mini_pupper_navigation'
    LAUNCH_FILE = 'planning.launch.py'

    def __init__(
        self,
        map_directory=None,
        log_path=None,
        process_factory=None,
        process_finder=None,
        identity_checker=None,
        get_process_group=None,
        signal_process_group=None,
    ):
        home = Path.home()

        self._map_directory = Path(
            map_directory
            or home
            / 'robot_maps'
            / 'mayday_supervised_route_03'
        )
        self._map_yaml = (
            self._map_directory
            / 'mayday_supervised_route_03.yaml'
        )
        self._checksum_manifest = (
            self._map_directory / 'SHA256SUMS'
        )
        self._log_path = Path(
            log_path
            or home
            / 'robot_bridge'
            / 'logs'
            / 'planning_control.log'
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

    @property
    def command(self):
        """Return the fixed planning-only launch command."""
        return [
            str(self.ROS2_EXECUTABLE),
            'launch',
            self.PACKAGE,
            self.LAUNCH_FILE,
            f'map:={self._map_yaml}',
            'use_sim_time:=false',
            'autostart:=true',
            'initial_pose_x:=0.0',
            'initial_pose_y:=0.0',
            'initial_pose_yaw:=0.0',
        ]

    def _refresh_locked(self):
        if (
            self._process is not None
            and self._process.poll() is not None
        ):
            self._process = None
            self._started_at = None

    def snapshot(self):
        """Return process ownership without changing runtime."""
        with self._lock:
            self._refresh_locked()

            running = self._process is not None

            return {
                'running': running,
                'owned': running,
                'pid': (
                    int(self._process.pid)
                    if running
                    else None
                ),
                'state': (
                    'RUNNING'
                    if running
                    else 'STOPPED'
                ),
                'started_at': self._started_at,
                'last_stopped_at': self._last_stopped_at,
                'last_error': self._last_error,
                'map_yaml': str(self._map_yaml),
                'launch': (
                    f'{self.PACKAGE}/'
                    f'{self.LAUNCH_FILE}'
                ),
                'planning_enabled': True,
                'control_enabled': False,
                'motion_enabled': False,
                'execution_enabled': False,
                'controller_enabled': False,
                'navigator_enabled': False,
            }

    def start(self, timestamp):
        """Start one validated planning-only session."""
        with self._lock:
            self._refresh_locked()

            if self._process is not None:
                result = self.snapshot()
                result['action'] = 'ALREADY_RUNNING'
                result['started'] = False
                return result

            external = self._process_finder(None)

            if external:
                raise PlanningConflictError(
                    'Planning or Nav2 processes already '
                    'exist outside Robot Bridge ownership: '
                    + ', '.join(
                        str(pid)
                        for pid in external
                    )
                )

            self._verify_artifacts()

            self._log_path.parent.mkdir(
                parents=True,
                exist_ok=True,
            )

            log_handle = self._log_path.open(
                'ab',
                buffering=0,
            )

            try:
                process = self._process_factory(
                    self.command,
                    cwd=str(Path.home()),
                    stdin=subprocess.DEVNULL,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            finally:
                log_handle.close()

            if process.poll() is not None:
                self._last_error = (
                    'Planning launch exited immediately.'
                )
                raise PlanningControlError(
                    self._last_error
                )

            self._process = process
            self._started_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result['action'] = 'STARTED'
            result['started'] = True
            return result

    def stop(self, timestamp):
        """Stop only the planning session this object owns."""
        with self._lock:
            self._refresh_locked()

            if self._process is None:
                result = self.snapshot()
                result['action'] = 'ALREADY_STOPPED'
                result['stopped'] = False
                return result

            process = self._process
            pid = int(process.pid)

            if not self._identity_checker(pid):
                self._last_error = (
                    'Owned planning PID identity changed; '
                    'no signal was sent.'
                )
                raise PlanningControlError(
                    self._last_error
                )

            process_group = self._get_process_group(pid)

            if process_group != pid:
                self._last_error = (
                    'Planning is not its session leader; '
                    'no signal was sent.'
                )
                raise PlanningControlError(
                    self._last_error
                )

            self._signal_process_group(
                process_group,
                signal.SIGINT,
            )

            try:
                process.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                self._signal_process_group(
                    process_group,
                    signal.SIGTERM,
                )

                try:
                    process.wait(timeout=4.0)
                except subprocess.TimeoutExpired as exc:
                    self._last_error = (
                        'Planning did not stop after '
                        'SIGINT and SIGTERM.'
                    )
                    raise PlanningControlError(
                        self._last_error
                    ) from exc

            self._process = None
            self._started_at = None
            self._last_stopped_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result['action'] = 'STOPPED'
            result['stopped'] = True
            result['stopped_pid'] = pid
            return result

    def shutdown(self):
        """Best-effort shutdown for Robot Bridge process exit."""
        try:
            self.stop('ROBOT_BRIDGE_SHUTDOWN')
        except Exception:
            pass

    def _verify_artifacts(self):
        if not self.ROS2_EXECUTABLE.is_file():
            raise PlanningControlError(
                'ROS 2 executable is unavailable.'
            )

        if not self._map_yaml.is_file():
            raise PlanningControlError(
                'Validated saved-map YAML is unavailable.'
            )

        if not self._checksum_manifest.is_file():
            raise PlanningControlError(
                'Saved-map checksum manifest is unavailable.'
            )

        expected = {}

        for raw_line in self._checksum_manifest.read_text(
            encoding='utf-8',
        ).splitlines():
            fields = raw_line.strip().split()

            if len(fields) != 2:
                continue

            digest, filename = fields
            normalized_name = Path(
                filename.lstrip('*')
            ).name
            expected[normalized_name] = digest

        required = (
            'mayday_supervised_route_03.pbstream',
            'mayday_supervised_route_03.yaml',
            'mayday_supervised_route_03.pgm',
        )

        for filename in required:
            path = self._map_directory / filename
            expected_digest = expected.get(filename)

            if (
                expected_digest is None
                or not path.is_file()
            ):
                raise PlanningControlError(
                    f'Validated map artifact missing: '
                    f'{filename}'
                )

            actual_digest = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()

            if actual_digest != expected_digest:
                raise PlanningControlError(
                    f'Validated map checksum failed: '
                    f'{filename}'
                )

    @classmethod
    def _is_planning_command(cls, command):
        normalized = ' '.join(
            str(command).split()
        )

        markers = (
            'mini_pupper_navigation',
            cls.LAUNCH_FILE,
        )

        if all(marker in normalized for marker in markers):
            return True

        child_markers = (
            'localization.launch.py',
            '/nav2_amcl/amcl',
            '/nav2_map_server/map_server',
            '/nav2_lifecycle_manager/lifecycle_manager',
            '/nav2_planner/planner_server',
            '/nav2_controller/controller_server',
            '/nav2_bt_navigator/bt_navigator',
            '/nav2_behaviors/behavior_server',
            '/nav2_velocity_smoother/velocity_smoother',
            'cartographer',
        )

        return any(
            marker in normalized
            for marker in child_markers
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

            if cls._is_planning_command(command):
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
