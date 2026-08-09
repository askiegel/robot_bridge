#!/usr/bin/env python3

import hashlib
import signal
import subprocess
from pathlib import Path

import pytest

from planning_control import (
    PlanningConflictError,
    PlanningControl,
    PlanningControlError,
)


class FakeProcess:
    def __init__(self, pid=42001):
        self.pid = pid
        self.return_code = None

    def poll(self):
        return self.return_code

    def wait(self, timeout=None):
        self.return_code = 0
        return self.return_code


def create_map(tmp_path):
    directory = (
        tmp_path / "mayday_supervised_route_03"
    )
    directory.mkdir()

    artifacts = {
        "mayday_supervised_route_03.pbstream": b"pbstream",
        "mayday_supervised_route_03.yaml": (
            b"image: mayday_supervised_route_03.pgm\n"
        ),
        "mayday_supervised_route_03.pgm": (
            b"P5\n1 1\n255\n\xfe"
        ),
    }

    manifest = []

    for name, content in artifacts.items():
        artifact = directory / name
        artifact.write_bytes(content)
        manifest.append(
            f"{hashlib.sha256(content).hexdigest()}  {name}"
        )

    (
        directory / "SHA256SUMS"
    ).write_text(
        "\n".join(manifest) + "\n",
        encoding="utf-8",
    )

    return directory


def make_control(
    tmp_path,
    *,
    external=None,
    identity=True,
):
    map_directory = create_map(tmp_path)
    ros2 = tmp_path / "ros2"
    ros2.write_text("#!/bin/sh\n", encoding="utf-8")

    PlanningControl.ROS2_EXECUTABLE = ros2

    process = FakeProcess()
    signals = []
    calls = []

    def process_factory(*args, **kwargs):
        calls.append((args, kwargs))
        return process

    control = PlanningControl(
        map_directory=map_directory,
        log_path=tmp_path / "planning.log",
        process_factory=process_factory,
        process_finder=lambda owned: list(
            external or []
        ),
        identity_checker=lambda pid: identity,
        get_process_group=lambda pid: pid,
        signal_process_group=lambda pgid, sig: (
            signals.append((pgid, sig))
        ),
    )

    return control, process, calls, signals


def test_fixed_planning_only_command(tmp_path):
    control, _, _, _ = make_control(tmp_path)
    command = control.command
    joined = " ".join(command)

    assert command[0].endswith("/ros2")
    assert "mini_pupper_navigation" in command
    assert "planning.launch.py" in command
    assert "mayday_supervised_route_03.yaml" in joined
    assert "use_sim_time:=false" in command
    assert "autostart:=true" in command

    for forbidden in (
        "controller_server",
        "bt_navigator",
        "velocity_smoother",
        "cmd_vel",
    ):
        assert forbidden not in joined


def test_stopped_snapshot_disables_motion(tmp_path):
    control, _, _, _ = make_control(tmp_path)
    snapshot = control.snapshot()

    assert snapshot["running"] is False
    assert snapshot["owned"] is False
    assert snapshot["planning_enabled"] is True
    assert snapshot["motion_enabled"] is False
    assert snapshot["execution_enabled"] is False
    assert snapshot["controller_enabled"] is False
    assert snapshot["navigator_enabled"] is False


def test_start_owns_planning_session(tmp_path):
    control, process, calls, _ = make_control(tmp_path)

    result = control.start("START_TIME")

    assert result["started"] is True
    assert result["running"] is True
    assert result["owned"] is True
    assert result["pid"] == process.pid
    assert result["started_at"] == "START_TIME"
    assert calls[0][1]["start_new_session"] is True
    assert calls[0][1]["stdin"] is subprocess.DEVNULL


def test_duplicate_start_is_idempotent(tmp_path):
    control, _, _, _ = make_control(tmp_path)

    first = control.start("FIRST")
    second = control.start("SECOND")

    assert first["started"] is True
    assert second["started"] is False
    assert second["action"] == "ALREADY_RUNNING"
    assert second["started_at"] == "FIRST"


def test_external_navigation_is_blocked(tmp_path):
    control, _, _, _ = make_control(
        tmp_path,
        external=[1234],
    )

    with pytest.raises(
        PlanningConflictError,
        match="outside Robot Bridge ownership",
    ):
        control.start("START_TIME")


def test_bad_map_checksum_is_blocked(tmp_path):
    control, _, _, _ = make_control(tmp_path)

    (
        control._map_directory
        / "mayday_supervised_route_03.pgm"
    ).write_bytes(b"changed")

    with pytest.raises(
        PlanningControlError,
        match="checksum failed",
    ):
        control.start("START_TIME")


def test_stop_signals_owned_group(tmp_path):
    control, process, _, signals = make_control(
        tmp_path
    )
    control.start("START_TIME")

    result = control.stop("STOP_TIME")

    assert signals == [
        (process.pid, signal.SIGINT),
    ]
    assert result["stopped"] is True
    assert result["stopped_pid"] == process.pid
    assert result["running"] is False
    assert result["last_stopped_at"] == "STOP_TIME"


def test_changed_identity_is_not_signaled(tmp_path):
    control, _, _, signals = make_control(
        tmp_path,
        identity=False,
    )
    control.start("START_TIME")

    with pytest.raises(
        PlanningControlError,
        match="identity changed",
    ):
        control.stop("STOP_TIME")

    assert signals == []


def test_process_detection_includes_localization():
    assert PlanningControl._is_planning_command(
        "/opt/ros/humble/bin/ros2 launch "
        "mini_pupper_navigation planning.launch.py"
    )
    assert PlanningControl._is_planning_command(
        "/opt/ros/humble/bin/ros2 launch "
        "mini_pupper_navigation localization.launch.py"
    )
    assert PlanningControl._is_planning_command(
        "/opt/ros/humble/lib/nav2_planner/"
        "planner_server"
    )
    assert PlanningControl._is_planning_command(
        "/opt/ros/humble/lib/nav2_amcl/amcl"
    )
    assert not PlanningControl._is_planning_command(
        "/usr/bin/python3 app.py"
    )


def test_application_has_lifecycle_routes_only():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    assert (
        '@app.route("/planning/status", methods=["GET"])'
        in source
    )
    assert (
        '@app.route("/planning/start", methods=["POST"])'
        in source
    )
    assert (
        '@app.route("/planning/stop", methods=["POST"])'
        in source
    )
    assert (
        '@app.route("/planning/compute-path"'
        not in source
    )


def test_planning_start_has_safety_guards():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    control = source[
        source.index(
            '@app.route("/planning/start"'
        ):
        source.index(
            '@app.route("/planning/stop"'
        )
    ]

    assert "stop_robot()" in control
    assert (
        "mapping_control.snapshot().get('running')"
        in control
    )
    assert (
        "localization_control.snapshot().get('running')"
        in control
    )


def test_planning_blocks_other_runtime_starts():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    mapping = source[
        source.index(
            '@app.route("/mapping/start"'
        ):
        source.index(
            '@app.route('
            '\n    "/mapping/save-candidate"'
        )
    ]

    localization = source[
        source.index(
            '@app.route("/localization/start"'
        ):
        source.index(
            '@app.route("/localization/stop"'
        )
    ]

    marker = (
        "planning_control.snapshot().get('running')"
    )

    assert marker in mapping
    assert marker in localization
