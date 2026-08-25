#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#

import os
import signal
import subprocess
import threading
from pathlib import Path


class MappingNavigationControlError(RuntimeError):
    """Live-mapping guarded navigation could not be controlled."""


class MappingNavigationConflictError(
    MappingNavigationControlError
):
    """Another incompatible navigation runtime exists."""


class MappingNavigationControl:
    """
    Own Nav2 execution while Cartographer remains authoritative.

    Cartographer must already own the live mapping runtime.
    This controller starts planner/controller/BT navigation only.
    It never starts AMCL or map_server.
    """

    ROS2_EXECUTABLE = Path("/opt/ros/humble/bin/ros2")
    PACKAGE = "mini_pupper_navigation"
    LAUNCH_FILE = "mapping_navigation.launch.py"

    MAXIMUM_GOAL_DISTANCE_METERS = 0.50
    MAXIMUM_EXECUTION_SECONDS = 25.0

    def __init__(
        self,
        mapping_state_provider,
        launch_path=None,
        log_path=None,
        ros2_executable=None,
        process_factory=None,
        process_finder=None,
        identity_checker=None,
        get_process_group=None,
        signal_process_group=None,
        action_server_ready_provider=None,
    ):
        if not callable(mapping_state_provider):
            raise TypeError(
                "mapping_state_provider must be callable."
            )

        home = Path.home()

        self._mapping_state_provider = (
            mapping_state_provider
        )

        self._ros2_executable = Path(
            ros2_executable
            or self.ROS2_EXECUTABLE
        )

        self._launch_path = Path(
            launch_path
            or home
            / "ros2_ws"
            / "install"
            / self.PACKAGE
            / "share"
            / self.PACKAGE
            / "launch"
            / self.LAUNCH_FILE
        )

        self._log_path = Path(
            log_path
            or home
            / "robot_bridge"
            / "logs"
            / "mapping_navigation_control.log"
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
        self._action_server_ready_provider = (
            action_server_ready_provider
            or (lambda: False)
        )

        if not callable(
            self._action_server_ready_provider
        ):
            raise TypeError(
                "action_server_ready_provider "
                "must be callable."
            )

        self._lock = threading.RLock()
        self._process = None
        self._started_at = None
        self._last_stopped_at = None
        self._last_error = None

    @property
    def command(self):
        """Return the fixed mapping-time Nav2 command."""
        return [
            str(self._ros2_executable),
            "launch",
            self.PACKAGE,
            self.LAUNCH_FILE,
            "use_sim_time:=false",
            "autostart:=true",
        ]

    def _mapping_snapshot(self):
        try:
            state = self._mapping_state_provider()
        except Exception as exc:
            return {
                "running": False,
                "owned": False,
                "error": str(exc),
            }

        if not isinstance(state, dict):
            return {
                "running": False,
                "owned": False,
                "error": (
                    "Mapping state provider returned "
                    "an invalid value."
                ),
            }

        return dict(state)

    @staticmethod
    def _mapping_ready(mapping):
        return bool(
            mapping.get("running")
            and mapping.get("owned")
        )

    def _refresh_locked(self):
        if (
            self._process is not None
            and self._process.poll() is not None
        ):
            self._process = None
            self._started_at = None

    def snapshot(self):
        """Return ownership without changing runtime."""
        with self._lock:
            self._refresh_locked()

            mapping = self._mapping_snapshot()
            mapping_ready = self._mapping_ready(mapping)

            running = self._process is not None

            action_server_ready = False

            if running and mapping_ready:
                try:
                    action_server_ready = bool(
                        self._action_server_ready_provider()
                    )
                except Exception:
                    action_server_ready = False

            enabled = bool(
                running
                and mapping_ready
                and action_server_ready
            )

            if enabled:
                state = "RUNNING"
            elif running and mapping_ready:
                state = "STARTING"
            elif running:
                state = "MAPPING_UNAVAILABLE"
            else:
                state = "STOPPED"

            return {
                "running": running,
                "owned": running,
                "pid": (
                    int(self._process.pid)
                    if running
                    else None
                ),
                "state": state,
                "mode": "live_mapping",
                "started_at": self._started_at,
                "last_stopped_at": self._last_stopped_at,
                "last_error": self._last_error,
                "launch": (
                    f"{self.PACKAGE}/"
                    f"{self.LAUNCH_FILE}"
                ),
                "mapping_required": True,
                "mapping_ready": mapping_ready,
                "mapping": mapping,
                "map_source": "live_cartographer_map",
                "pose_source": "cartographer_tf",
                "action_server_ready": (
                    action_server_ready
                ),
                "planning_enabled": enabled,
                "navigation_enabled": enabled,
                "control_enabled": enabled,
                "motion_enabled": enabled,
                "execution_enabled": enabled,
                "goal_submission_enabled": enabled,
                "controller_enabled": enabled,
                "navigator_enabled": enabled,
                "maximum_goal_distance_meters": (
                    self.MAXIMUM_GOAL_DISTANCE_METERS
                ),
                "maximum_execution_seconds": (
                    self.MAXIMUM_EXECUTION_SECONDS
                ),
            }

    def start(self, timestamp):
        """
        Start Nav2 execution only while owned mapping is active.
        """
        with self._lock:
            self._refresh_locked()

            if self._process is not None:
                result = self.snapshot()
                result["action"] = "ALREADY_RUNNING"
                result["started"] = False
                return result

            mapping = self._mapping_snapshot()

            if not self._mapping_ready(mapping):
                raise MappingNavigationConflictError(
                    "Owned live Cartographer mapping "
                    "runtime is required."
                )

            external = self._process_finder(None)

            if external:
                raise MappingNavigationConflictError(
                    "Another navigation or Nav2 process "
                    "already exists outside Robot Bridge "
                    "ownership: "
                    + ", ".join(
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
                "ab",
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
                    "Mapping navigation launch "
                    "exited immediately."
                )
                raise MappingNavigationControlError(
                    self._last_error
                )

            self._process = process
            self._started_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result["action"] = "STARTED"
            result["started"] = True
            return result

    def stop(self, timestamp):
        """
        Stop only the mapping-navigation session we own.

        Stopping this controller does not stop Cartographer.
        """
        with self._lock:
            self._refresh_locked()

            if self._process is None:
                result = self.snapshot()
                result["action"] = "ALREADY_STOPPED"
                result["stopped"] = False
                return result

            process = self._process
            pid = int(process.pid)

            if not self._identity_checker(pid):
                self._last_error = (
                    "Owned mapping-navigation PID "
                    "identity changed; no signal was sent."
                )
                raise MappingNavigationControlError(
                    self._last_error
                )

            process_group = self._get_process_group(pid)

            if process_group != pid:
                self._last_error = (
                    "Mapping navigation is not its "
                    "session leader; no signal was sent."
                )
                raise MappingNavigationControlError(
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
                        "Mapping navigation did not stop "
                        "after SIGINT and SIGTERM."
                    )
                    raise (
                        MappingNavigationControlError(
                            self._last_error
                        )
                    ) from exc

            self._process = None
            self._started_at = None
            self._last_stopped_at = timestamp
            self._last_error = None

            result = self.snapshot()
            result["action"] = "STOPPED"
            result["stopped"] = True
            result["stopped_pid"] = pid
            return result

    def shutdown(self):
        """Best-effort owned-process shutdown."""
        try:
            self.stop("ROBOT_BRIDGE_SHUTDOWN")
        except Exception:
            pass

    def _verify_artifacts(self):
        if not self._ros2_executable.is_file():
            raise MappingNavigationControlError(
                "ROS 2 executable is unavailable."
            )

        if not self._launch_path.is_file():
            raise MappingNavigationControlError(
                "Installed mapping navigation launch "
                "is unavailable."
            )

    @classmethod
    def _is_conflicting_command(cls, command):
        """
        Detect incompatible Nav2/localization processes.

        Cartographer is intentionally NOT a conflict because
        it owns map localization in this mode.
        """
        normalized = " ".join(
            str(command).split()
        )

        own_launch_markers = (
            cls.PACKAGE,
            cls.LAUNCH_FILE,
        )

        if all(
            marker in normalized
            for marker in own_launch_markers
        ):
            return True

        conflicting_markers = (
            "guarded_navigation.launch.py",
            "planning.launch.py",
            "localization.launch.py",
            "/nav2_amcl/amcl",
            "/nav2_map_server/map_server",
            "/nav2_lifecycle_manager/lifecycle_manager",
            "/nav2_planner/planner_server",
            "/nav2_controller/controller_server",
            "/nav2_bt_navigator/bt_navigator",
            "/nav2_behaviors/behavior_server",
            "/nav2_velocity_smoother/velocity_smoother",
            "latest_tf_relay.py",
        )

        return any(
            marker in normalized
            for marker in conflicting_markers
        )

    @classmethod
    def _find_external_processes(cls, owned_pid):
        found = []

        for process_directory in Path("/proc").iterdir():
            if not process_directory.name.isdigit():
                continue

            pid = int(process_directory.name)

            if (
                owned_pid is not None
                and pid == owned_pid
            ):
                continue

            try:
                command = (
                    process_directory
                    / "cmdline"
                ).read_bytes().replace(
                    b"\0",
                    b" ",
                ).decode(
                    "utf-8",
                    errors="replace",
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
                Path("/proc")
                / str(int(pid))
                / "cmdline"
            ).read_bytes().replace(
                b"\0",
                b" ",
            ).decode(
                "utf-8",
                errors="replace",
            )
        except (
            OSError,
            PermissionError,
            ValueError,
        ):
            return False

        return (
            "ros2" in command
            and "launch" in command
            and cls.PACKAGE in command
            and cls.LAUNCH_FILE in command
        )
