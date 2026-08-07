#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import os
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

    def __init__(
        self,
        launch_path=None,
        log_path=None,
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
        """Return the fixed headless mapping-only command."""
        return [
            str(self.ROS2_EXECUTABLE),
            'launch',
            self.PACKAGE,
            self.LAUNCH_FILE,
            'use_sim_time:=false',
            'use_rviz:=false',
        ]

    def _refresh_locked(self):
        if (
            self._process is not None
            and self._process.poll() is not None
        ):
            self._process = None
            self._started_at = None

    def snapshot(self):
        """Return mapping ownership without changing runtime."""
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
                'launch': (
                    f'{self.PACKAGE}/'
                    f'{self.LAUNCH_FILE}'
                ),
                'headless': True,
                'planning_enabled': False,
                'control_enabled': False,
                'map_save_enabled': False,
                'validated_map_mutable': False,
            }

    def start(self, timestamp):
        """Start one fixed headless mapping-only session."""
        with self._lock:
            self._refresh_locked()

            if self._process is not None:
                result = self.snapshot()
                result['action'] = 'ALREADY_RUNNING'
                result['started'] = False
                return result

            external = self._process_finder(None)

            if external:
                raise MappingConflictError(
                    'Mapping, localization, or Nav2 processes '
                    'already exist outside Robot Bridge '
                    'ownership: '
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
                    'Mapping launch exited immediately.'
                )
                raise MappingControlError(
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
        """Stop only the mapping session this object owns."""
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
                    'Owned mapping PID identity changed; '
                    'no signal was sent.'
                )
                raise MappingControlError(
                    self._last_error
                )

            process_group = self._get_process_group(pid)

            if process_group != pid:
                self._last_error = (
                    'Mapping is not its session leader; '
                    'no signal was sent.'
                )
                raise MappingControlError(
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
                        'Mapping did not stop after '
                        'SIGINT and SIGTERM.'
                    )
                    raise MappingControlError(
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
        """Best-effort shutdown when Robot Bridge exits."""
        try:
            self.stop('ROBOT_BRIDGE_SHUTDOWN')
        except Exception:
            pass

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
