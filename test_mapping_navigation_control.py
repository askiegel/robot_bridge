#!/usr/bin/env python3

import signal
from pathlib import Path

import pytest

from mapping_navigation_control import (
    MappingNavigationConflictError,
    MappingNavigationControl,
)


class FakeProcess:
    def __init__(self, pid=4321):
        self.pid = pid
        self.returncode = None
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.wait_calls.append(timeout)
        self.returncode = 0
        return 0


def mapping_running():
    return {
        "running": True,
        "owned": True,
        "pid": 59303,
        "state": "RUNNING",
    }


def mapping_stopped():
    return {
        "running": False,
        "owned": False,
        "pid": None,
        "state": "STOPPED",
    }


def make_control(
    tmp_path,
    mapping_provider=mapping_running,
    process=None,
    process_finder=None,
    signals=None,
):
    ros2 = tmp_path / "ros2"
    ros2.write_text("", encoding="utf-8")

    launch = tmp_path / "mapping_navigation.launch.py"
    launch.write_text("", encoding="utf-8")

    fake_process = process or FakeProcess()

    if signals is None:
        signals = []

    return MappingNavigationControl(
        mapping_state_provider=mapping_provider,
        ros2_executable=ros2,
        launch_path=launch,
        log_path=tmp_path / "mapping_navigation.log",
        process_factory=lambda *args, **kwargs: fake_process,
        process_finder=process_finder or (lambda owned: []),
        identity_checker=lambda pid: True,
        get_process_group=lambda pid: pid,
        signal_process_group=(
            lambda pgid, sig: signals.append(
                (pgid, sig)
            )
        ),
    ), fake_process, signals


def test_command_uses_mapping_navigation_launch(tmp_path):
    control, _, _ = make_control(tmp_path)

    command = control.command

    assert "mapping_navigation.launch.py" in command
    assert "use_sim_time:=false" in command
    assert "autostart:=true" in command

    forbidden = (
        "map:=",
        "initial_pose_x",
        "initial_pose_y",
        "initial_pose_yaw",
    )

    for marker in forbidden:
        assert not any(
            marker in item
            for item in command
        )


def test_start_requires_owned_mapping(tmp_path):
    control, _, _ = make_control(
        tmp_path,
        mapping_provider=mapping_stopped,
    )

    with pytest.raises(
        MappingNavigationConflictError,
        match="Cartographer mapping",
    ):
        control.start("2026-08-23T19:00:00Z")


def test_start_exposes_live_mapping_mode(tmp_path):
    control, process, _ = make_control(tmp_path)

    result = control.start(
        "2026-08-23T19:00:00Z"
    )

    assert result["started"] is True
    assert result["running"] is True
    assert result["owned"] is True
    assert result["pid"] == process.pid
    assert result["mode"] == "live_mapping"

    assert result["mapping_ready"] is True
    assert result["map_source"] == (
        "live_cartographer_map"
    )
    assert result["pose_source"] == (
        "cartographer_tf"
    )

    assert result["execution_enabled"] is True
    assert result["goal_submission_enabled"] is True

    assert (
        result["maximum_goal_distance_meters"]
        == 0.50
    )
    assert (
        result["maximum_execution_seconds"]
        == 15.0
    )


def test_cartographer_is_not_a_conflict():
    assert (
        MappingNavigationControl
        ._is_conflicting_command(
            "/opt/ros/humble/lib/"
            "cartographer_ros/cartographer_node "
            "-configuration_directory /tmp"
        )
        is False
    )

    assert (
        MappingNavigationControl
        ._is_conflicting_command(
            "/opt/ros/humble/lib/"
            "cartographer_ros/"
            "cartographer_occupancy_grid_node"
        )
        is False
    )


def test_other_navigation_processes_are_conflicts():
    conflicts = (
        "ros2 launch mini_pupper_navigation "
        "guarded_navigation.launch.py",
        "/opt/ros/humble/lib/"
        "nav2_amcl/amcl",
        "/opt/ros/humble/lib/"
        "nav2_map_server/map_server",
        "/opt/ros/humble/lib/"
        "nav2_planner/planner_server",
        "/opt/ros/humble/lib/"
        "nav2_controller/controller_server",
        "/opt/ros/humble/lib/"
        "nav2_bt_navigator/bt_navigator",
        "python3 latest_tf_relay.py",
    )

    for command in conflicts:
        assert (
            MappingNavigationControl
            ._is_conflicting_command(command)
            is True
        )


def test_start_rejects_external_nav2(tmp_path):
    control, _, _ = make_control(
        tmp_path,
        process_finder=lambda owned: [9876],
    )

    with pytest.raises(
        MappingNavigationConflictError,
        match="9876",
    ):
        control.start("2026-08-23T19:00:00Z")


def test_stop_owns_only_its_process_group(tmp_path):
    process = FakeProcess(pid=4567)

    control, _, signals = make_control(
        tmp_path,
        process=process,
    )

    control.start("START")

    result = control.stop("STOP")

    assert result["stopped"] is True
    assert result["stopped_pid"] == 4567
    assert result["running"] is False

    assert signals == [
        (4567, signal.SIGINT),
    ]


def test_mapping_loss_disables_goal_submission(tmp_path):
    state = {
        "running": True,
        "owned": True,
    }

    control, _, _ = make_control(
        tmp_path,
        mapping_provider=lambda: dict(state),
    )

    control.start("START")

    state["running"] = False
    state["owned"] = False

    snapshot = control.snapshot()

    assert snapshot["running"] is True
    assert snapshot["state"] == (
        "MAPPING_UNAVAILABLE"
    )
    assert snapshot["mapping_ready"] is False
    assert snapshot["execution_enabled"] is False
    assert (
        snapshot["goal_submission_enabled"]
        is False
    )
