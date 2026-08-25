#!/usr/bin/env python3

import hashlib
import signal
import subprocess
from pathlib import Path

import pytest

from navigation_control import (
    NavigationConflictError,
    NavigationControl,
    NavigationControlError,
)


class FakeProcess:
    def __init__(self, pid=43001):
        self.pid = pid
        self.return_code = None
        self.wait_calls = []

    def poll(self):
        return self.return_code

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
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

    NavigationControl.ROS2_EXECUTABLE = ros2

    process = FakeProcess()
    signals = []
    calls = []

    def process_factory(*args, **kwargs):
        calls.append((args, kwargs))
        return process

    control = NavigationControl(
        map_directory=map_directory,
        log_path=tmp_path / "navigation.log",
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


def test_fixed_guarded_navigation_command(tmp_path):
    control, _, _, _ = make_control(tmp_path)
    command = control.command
    joined = " ".join(command)

    assert command[0].endswith("/ros2")
    assert "mini_pupper_navigation" in command
    assert "guarded_navigation.launch.py" in command
    assert "mayday_supervised_route_03.yaml" in joined
    assert "use_sim_time:=false" in command
    assert "autostart:=true" in command

    assert "navigation.launch.py" not in command

    for forbidden in (
        "rviz2",
        "goal_x",
        "goal_y",
        "NavigateToPose",
    ):
        assert forbidden not in joined


def test_snapshot_exposes_runtime_without_goal_execution(
    tmp_path,
):
    control, _, _, _ = make_control(tmp_path)
    snapshot = control.snapshot()

    assert snapshot["running"] is False
    assert snapshot["owned"] is False
    assert snapshot["planning_enabled"] is True
    assert snapshot["navigation_enabled"] is True
    assert snapshot["control_enabled"] is True
    assert snapshot["motion_enabled"] is True
    assert snapshot["execution_enabled"] is True
    assert snapshot["goal_submission_enabled"] is True
    assert snapshot["controller_enabled"] is True
    assert snapshot["navigator_enabled"] is True
    assert snapshot["maximum_goal_distance_meters"] == 0.50
    assert snapshot["maximum_execution_seconds"] == 25.0


def test_start_owns_guarded_navigation_session(tmp_path):
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


def test_external_nav2_is_blocked(tmp_path):
    control, _, _, _ = make_control(
        tmp_path,
        external=[1234],
    )

    with pytest.raises(
        NavigationConflictError,
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
        NavigationControlError,
        match="checksum failed",
    ):
        control.start("START_TIME")


def test_stop_signals_only_owned_group(tmp_path):
    control, process, _, signals = make_control(
        tmp_path
    )
    control.start("START_TIME")

    result = control.stop("STOP_TIME")

    assert signals == [
        (process.pid, signal.SIGINT),
    ]
    assert process.wait_calls == [12.0]
    assert result["stopped"] is True
    assert result["stopped_pid"] == process.pid
    assert result["running"] is False


def test_changed_identity_is_not_signaled(tmp_path):
    control, _, _, signals = make_control(
        tmp_path,
        identity=False,
    )
    control.start("START_TIME")

    with pytest.raises(
        NavigationControlError,
        match="identity changed",
    ):
        control.stop("STOP_TIME")

    assert signals == []


def test_process_detection_includes_all_nav2_modes():
    commands = (
        (
            "/opt/ros/humble/bin/ros2 launch "
            "mini_pupper_navigation "
            "guarded_navigation.launch.py"
        ),
        (
            "/opt/ros/humble/bin/ros2 launch "
            "mini_pupper_navigation "
            "planning.launch.py"
        ),
        "/opt/ros/humble/lib/nav2_amcl/amcl",
        (
            "/opt/ros/humble/lib/nav2_controller/"
            "controller_server"
        ),
        (
            "/opt/ros/humble/lib/nav2_bt_navigator/"
            "bt_navigator"
        ),
    )

    for command in commands:
        assert NavigationControl._is_navigation_command(
            command
        )

    assert not NavigationControl._is_navigation_command(
        "/usr/bin/python3 app.py"
    )


def test_application_has_guarded_navigation_routes():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    for route in (
        '@app.route("/navigation/status", methods=["GET"])',
        '@app.route("/navigation/start", methods=["POST"])',
        '@app.route("/navigation/goal", methods=["POST"])',
        '@app.route("/navigation/stop", methods=["POST"])',
    ):
        assert route in source

    for forbidden in (
        '"/navigation/navigate"',
        '"/navigation/execute"',
        "send_goal_async",
    ):
        assert forbidden not in source


def test_navigation_start_requires_safety_zero_and_exclusivity():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    control = source[
        source.index(
            '@app.route("/navigation/start"'
        ):
        source.index(
            '@app.route("/navigation/stop"'
        )
    ]

    assert "stop_robot()" in control
    assert "mapping_control.snapshot()" in control
    assert "planning_control.snapshot()" in control
    assert "localization_control.snapshot()" in control
    assert "localization_telemetry.clear()" in control


def test_navigation_start_requires_exact_scan_time_tf_preflight():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    control = source[
        source.index(
            '@app.route("/navigation/start"'
        ):
        source.index(
            '@app.route("/navigation/stop"'
        )
    ]

    node = source[
        source.index("class RobotBridgePublisher"):
        source.index("def ros_spin():")
    ]

    assert "'/odom/local'" in node
    assert "self.update_local_odom" in node
    assert "self.update_lidar" in node
    assert "message.header.stamp" in node
    assert "message.header.frame_id" in node
    assert "Time.from_msg(scan_stamp)" in node
    assert "lookup_transform(" in node
    assert "'odom'," in node
    assert (
        "publisher_node.navigation_start_preflight()"
        in control
    )
    assert (
        control.index(
            "publisher_node.navigation_start_preflight()"
        )
        < control.index(
            "navigation_control.start(timestamp)"
        )
    )
    assert "'preflight': preflight" in control
    assert (
        "NAVIGATION_PREFLIGHT_MAX_SENSOR_AGE_SECONDS"
        in source
    )
    assert (
        "NAVIGATION_PREFLIGHT_TF_TIMEOUT_SECONDS"
        in source
    )
    assert (
        "NAVIGATION_PREFLIGHT_TF_TIMEOUT_SECONDS = 1.00"
        in source
    )
    assert "Time.from_msg(scan_stamp)" in node


def test_other_modes_block_navigation_overlap():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    marker = (
        "navigation_control.snapshot().get('running')"
    )

    mapping = source[
        source.index('@app.route("/mapping/start"'):
        source.index(
            '@app.route('
            '\n    "/mapping/save-candidate"'
        )
    ]

    planning = source[
        source.index('@app.route("/planning/start"'):
        source.index(
            '@app.route('
            '\n    "/planning/initialize-localization"'
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

    assert marker in mapping
    assert marker in planning
    assert marker in localization


def test_global_shutdown_registers_navigation_cleanup():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    assert (
        "atexit.register(navigation_control.shutdown)"
        in source
    )


def test_robot_bridge_processes_tf_without_telemetry_backlog():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    assert (
        "from rclpy.executors import MultiThreadedExecutor"
        in source
    )
    assert "SingleThreadedExecutor" not in source
    assert "executor = MultiThreadedExecutor(" in source
    assert "num_threads=2" in source

    tf_start = source.index(
        "        self.navigation_tf_buffer = Buffer()"
    )
    tf_end = source.index(
        "        self.planning_path_service =",
        tf_start,
    )
    tf_listener = source[tf_start:tf_end]

    assert "navigation_tf_qos = QoSProfile(" in tf_listener
    assert "history=HistoryPolicy.KEEP_LAST" in tf_listener
    assert "depth=1" in tf_listener
    assert (
        "reliability=ReliabilityPolicy.BEST_EFFORT"
        in tf_listener
    )
    assert (
        "durability=DurabilityPolicy.VOLATILE"
        in tf_listener
    )
    assert "spin_thread=False" in tf_listener
    assert "qos=navigation_tf_qos" in tf_listener

    assert "Time.from_msg(scan_stamp)" in source
    assert "lookup_transform(" in source

def test_navigation_goal_refreshes_pose_atomically():
    source = open(
        "app.py",
        encoding="utf-8",
    ).read()

    start = source.index(
        "    def execute_navigation_goal(self, payload):"
    )
    end = source.index(
        "    def cancel_navigation_goal(self):",
        start,
    )
    method = source[start:end]

    refresh = method.index(
        "self.refresh_planning_localization_pose()"
    )
    snapshot = method.index(
        "localization_telemetry.snapshot()"
    )
    execution = method.index(
        "self.navigation_goal_service.execute("
    )

    assert refresh < snapshot < execution
    assert "for attempt in range(60):" in method
    assert "time.sleep(0.05)" in method
    assert (
        "pose_snapshot.get('available') is True"
        in method
    )
    assert (
        "except PlanningLocalizationError as exc:"
        in method
    )
    assert "reinitialize_global_localization" not in method
    assert "NavigateToPose" not in method
    assert "cmd_vel" not in method

