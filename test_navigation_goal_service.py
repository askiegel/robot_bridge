#!/usr/bin/env python3

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from navigation_goal_service import (
    NavigationGoalCancelledError,
    NavigationGoalConflictError,
    NavigationGoalError,
    NavigationGoalService,
    NavigationGoalTimeoutError,
    NavigationGoalUnavailableError,
    NavigationGoalValidationError,
)


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class NeverFuture:
    def add_done_callback(self, callback):
        self.callback = callback


class FakeClock:
    def now(self):
        return SimpleNamespace(
            to_msg=lambda: SimpleNamespace(
                sec=123,
                nanosec=456,
            )
        )


class FakeNode:
    def get_clock(self):
        return FakeClock()


class FakeGoalHandle:
    def __init__(
        self,
        *,
        accepted=True,
        status=GoalStatus.STATUS_SUCCEEDED,
        result_future=None,
    ):
        self.accepted = accepted
        self.status = status
        self.cancelled = False
        self.result_future = result_future

    def get_result_async(self):
        if self.result_future is not None:
            return self.result_future

        return FakeFuture(
            SimpleNamespace(
                status=self.status,
                result=SimpleNamespace(),
            )
        )

    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture(SimpleNamespace())


class FakeActionClient:
    def __init__(
        self,
        node,
        action_type,
        action_name,
    ):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
        self.available = True
        self.goal_handle = FakeGoalHandle()
        self.sent_goal = None

    def wait_for_server(self, timeout_sec):
        self.server_timeout = timeout_sec
        return self.available

    def send_goal_async(self, goal):
        self.sent_goal = goal
        return FakeFuture(self.goal_handle)


def fresh_pose(x=1.0, y=2.0, age=0.1):
    return {
        "available": True,
        "status": "READY",
        "age_seconds": age,
        "received_at": "2026-08-14T00:00:00+00:00",
        "pose": {
            "frame_id": "map",
            "position": {
                "x": x,
                "y": y,
                "z": 0.0,
            },
        },
    }


def make_service():
    clients = []

    def factory(node, action_type, action_name):
        client = FakeActionClient(
            node,
            action_type,
            action_name,
        )
        clients.append(client)
        return client

    service = NavigationGoalService(
        FakeNode(),
        action_client_factory=factory,
    )

    return service, clients[0]


@pytest.mark.parametrize(
    "payload,error",
    (
        (None, "JSON object"),
        ({}, "goal_x is required"),
        ({"goal_x": 1.0}, "goal_y is required"),
        (
            {"goal_x": True, "goal_y": 2.0},
            "goal_x must be a finite number",
        ),
        (
            {"goal_x": 1.0, "goal_y": math.inf},
            "goal_y must be a finite number",
        ),
        (
            {
                "goal_x": 1.0,
                "goal_y": 2.0,
                "behavior_tree": "unsafe",
            },
            "Unsupported request fields",
        ),
        (
            {
                "goal_x": 1.0,
                "goal_y": 2.0,
                "retries": 1,
            },
            "Unsupported request fields",
        ),
    ),
)
def test_invalid_requests_are_rejected(payload, error):
    with pytest.raises(
        NavigationGoalValidationError,
        match=error,
    ):
        NavigationGoalService.validate_request(payload)


@pytest.mark.parametrize(
    "snapshot,error",
    (
        (None, "Fresh localization"),
        (
            {"available": False},
            "Fresh localization",
        ),
        (
            fresh_pose(age=3.01),
            "pose is stale",
        ),
        (
            {
                **fresh_pose(),
                "pose": {
                    **fresh_pose()["pose"],
                    "frame_id": "odom",
                },
            },
            "map frame",
        ),
    ),
)
def test_invalid_localization_is_rejected(snapshot, error):
    with pytest.raises(
        NavigationGoalValidationError,
        match=error,
    ):
        NavigationGoalService.validate_pose(snapshot)


def test_goal_beyond_half_meter_is_rejected():
    service, _ = make_service()

    with pytest.raises(
        NavigationGoalValidationError,
        match="0.50-meter limit",
    ):
        service.execute(
            {
                "goal_x": 1.501,
                "goal_y": 2.0,
                "goal_yaw": 0.0,
            },
            fresh_pose(),
        )


def test_fixed_guarded_goal_succeeds():
    service, client = make_service()

    result = service.execute(
        {
            "goal_x": 1.15,
            "goal_y": 2.10,
            "goal_yaw": math.pi / 4.0,
        },
        fresh_pose(),
    )

    assert result["status"] == "NAVIGATION_SUCCEEDED"
    assert result["executed"] is True
    assert result["bounded"] is True
    assert result["retries_requested"] == 0
    assert result["recoveries_requested"] == 0
    assert result[
        "maximum_goal_distance_meters"
    ] == 0.50
    assert result[
        "maximum_execution_seconds"
    ] == 15.0
    assert result[
        "requested_distance_meters"
    ] == pytest.approx(math.hypot(0.15, 0.10))

    goal = client.sent_goal

    assert client.action_type is NavigateToPose
    assert client.action_name == "/navigate_to_pose"
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.header.stamp.sec == 123
    assert goal.pose.header.stamp.nanosec == 456
    assert goal.pose.pose.position.x == 1.15
    assert goal.pose.pose.position.y == 2.10
    assert goal.behavior_tree.endswith(
        "mayday_guarded_navigate_to_pose.xml"
    )
    assert goal.pose.pose.orientation.z == pytest.approx(
        math.sin(math.pi / 8.0)
    )
    assert goal.pose.pose.orientation.w == pytest.approx(
        math.cos(math.pi / 8.0)
    )


def test_action_server_ready_uses_existing_client_without_goal():
    service, client = make_service()

    client.available = True

    assert service.action_server_ready() is True
    assert client.server_timeout == 0.0
    assert client.sent_goal is None

    client.available = False

    assert service.action_server_ready() is False
    assert client.server_timeout == 0.0
    assert client.sent_goal is None


def test_unavailable_action_server_is_blocked():
    service, client = make_service()
    client.available = False

    with pytest.raises(
        NavigationGoalUnavailableError,
        match="action server is unavailable",
    ):
        service.execute(
            {"goal_x": 1.1, "goal_y": 2.0},
            fresh_pose(),
        )


def test_rejected_goal_is_reported():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        accepted=False
    )

    with pytest.raises(
        NavigationGoalError,
        match="goal was rejected",
    ):
        service.execute(
            {"goal_x": 1.1, "goal_y": 2.0},
            fresh_pose(),
        )


def test_non_success_status_is_rejected():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        status=GoalStatus.STATUS_ABORTED
    )

    with pytest.raises(
        NavigationGoalError,
        match="did not succeed",
    ):
        service.execute(
            {"goal_x": 1.1, "goal_y": 2.0},
            fresh_pose(),
        )


def test_cancelled_status_is_reported():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        status=GoalStatus.STATUS_CANCELED
    )

    with pytest.raises(
        NavigationGoalCancelledError,
        match="was cancelled",
    ):
        service.execute(
            {"goal_x": 1.1, "goal_y": 2.0},
            fresh_pose(),
        )


def test_execution_timeout_cancels_goal(monkeypatch):
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        result_future=NeverFuture()
    )

    monkeypatch.setattr(
        service,
        "MAXIMUM_EXECUTION_SECONDS",
        0.01,
    )

    with pytest.raises(
        NavigationGoalTimeoutError,
        match="15-second execution limit",
    ):
        service.execute(
            {"goal_x": 1.1, "goal_y": 2.0},
            fresh_pose(),
        )

    assert client.goal_handle.cancelled is True


def test_concurrent_goal_is_rejected():
    service, _ = make_service()
    service._request_lock.acquire()

    try:
        with pytest.raises(
            NavigationGoalConflictError,
            match="already active",
        ):
            service.execute(
                {"goal_x": 1.1, "goal_y": 2.0},
                fresh_pose(),
            )
    finally:
        service._request_lock.release()


def test_cancel_active_signals_current_goal():
    service, client = make_service()
    service._request_lock.acquire()
    service._active_goal_handle = client.goal_handle

    try:
        result = service.cancel_active()
    finally:
        service._active_goal_handle = None
        service._request_lock.release()

    assert result == {
        "active": True,
        "cancel_requested": True,
        "cancel_signal_sent": True,
    }
    assert client.goal_handle.cancelled is True


def test_source_contains_no_retry_or_recovery_loop():
    source = Path(
        "navigation_goal_service.py"
    ).read_text(encoding="utf-8")

    assert "while " not in source
    assert "for attempt" not in source
    assert "BackUp" not in source
    assert "Spin" not in source
    assert "RecoveryNode" not in source
    assert "MAXIMUM_GOAL_DISTANCE_METERS = 0.50" in source
    assert "MAXIMUM_EXECUTION_SECONDS = 15.0" in source


def test_executor_is_connected_only_to_guarded_route():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    assert "NavigationGoalService" in source
    assert (
        '@app.route("/navigation/goal", '
        'methods=["POST"])'
        in source
    )
    assert "execute_navigation_goal" in source
    assert "localization_telemetry.snapshot()" in source

    for forbidden in (
        '@app.route("/navigation/navigate"',
        '@app.route("/navigation/execute"',
        "goal_distance",
        "execution_timeout",
        "behavior_tree = request",
    ):
        assert forbidden not in source
