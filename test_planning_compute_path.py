#!/usr/bin/env python3

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

from planning_path_service import (
    PlanningPathConflictError,
    PlanningPathError,
    PlanningPathService,
    PlanningPathUnavailableError,
    PlanningPathValidationError,
)


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


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


def pose(x, y, yaw=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(
                x=x,
                y=y,
                z=0.0,
            ),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
    )


class FakeGoalHandle:
    def __init__(
        self,
        *,
        accepted=True,
        poses=None,
        status=GoalStatus.STATUS_SUCCEEDED,
    ):
        self.accepted = accepted
        self.cancelled = False
        self._poses = (
            poses
            if poses is not None
            else [
                pose(0.0, 0.0),
                pose(0.3, 0.4),
            ]
        )
        self._status = status

    def get_result_async(self):
        result = SimpleNamespace(
            path=SimpleNamespace(
                poses=self._poses
            ),
            planning_time=SimpleNamespace(
                sec=0,
                nanosec=250_000_000,
            ),
        )

        wrapped = SimpleNamespace(
            status=self._status,
            result=result,
        )

        return FakeFuture(wrapped)

    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture(None)


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

    service = PlanningPathService(
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
            {"goal_x": True, "goal_y": 0.0},
            "goal_x must be a finite number",
        ),
        (
            {"goal_x": 0.0, "goal_y": "bad"},
            "goal_y must be a finite number",
        ),
        (
            {"goal_x": math.inf, "goal_y": 0.0},
            "goal_x must be a finite number",
        ),
        (
            {
                "goal_x": 0.0,
                "goal_y": 0.0,
                "planner_id": "unsafe",
            },
            "Unsupported request fields",
        ),
    ),
)
def test_invalid_requests_are_rejected(
    payload,
    error,
):
    with pytest.raises(
        PlanningPathValidationError,
        match=error,
    ):
        PlanningPathService.validate_request(
            payload
        )


def test_valid_request_uses_fixed_contract():
    values = (
        PlanningPathService.validate_request({
            "goal_x": 1,
            "goal_y": -2.5,
        })
    )

    assert values == {
        "goal_x": 1.0,
        "goal_y": -2.5,
        "goal_yaw": 0.0,
    }


def test_compute_returns_read_only_path():
    service, client = make_service()

    result = service.compute({
        "goal_x": 1.0,
        "goal_y": 2.0,
        "goal_yaw": math.pi / 2.0,
    })

    assert result["status"] == "PATH_READY"
    assert result["read_only"] is True
    assert result["executed"] is False
    assert result["motion_enabled"] is False
    assert result["frame_id"] == "map"
    assert result["planner_id"] == "GridBased"
    assert result["pose_count"] == 2
    assert result["length_meters"] == pytest.approx(
        0.5
    )
    assert result[
        "planning_time_seconds"
    ] == pytest.approx(0.25)

    goal = client.sent_goal

    assert goal.goal.header.frame_id == "map"
    assert goal.goal.pose.position.x == 1.0
    assert goal.goal.pose.position.y == 2.0
    assert goal.planner_id == "GridBased"
    assert goal.use_start is False


def test_unavailable_server_is_blocked():
    service, client = make_service()
    client.available = False

    with pytest.raises(
        PlanningPathUnavailableError,
        match="action server is unavailable",
    ):
        service.compute({
            "goal_x": 1.0,
            "goal_y": 2.0,
        })


def test_rejected_goal_is_reported():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        accepted=False
    )

    with pytest.raises(
        PlanningPathError,
        match="request was rejected",
    ):
        service.compute({
            "goal_x": 1.0,
            "goal_y": 2.0,
        })


def test_empty_path_is_rejected():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        poses=[]
    )

    with pytest.raises(
        PlanningPathError,
        match="no usable path",
    ):
        service.compute({
            "goal_x": 1.0,
            "goal_y": 2.0,
        })


def test_non_success_status_is_rejected():
    service, client = make_service()
    client.goal_handle = FakeGoalHandle(
        status=GoalStatus.STATUS_ABORTED
    )

    with pytest.raises(
        PlanningPathError,
        match="did not succeed",
    ):
        service.compute({
            "goal_x": 1.0,
            "goal_y": 2.0,
        })


def test_concurrent_request_is_rejected():
    service, _ = make_service()

    service._request_lock.acquire()

    try:
        with pytest.raises(
            PlanningPathConflictError,
            match="already active",
        ):
            service.compute({
                "goal_x": 1.0,
                "goal_y": 2.0,
            })
    finally:
        service._request_lock.release()


def test_application_route_is_post_only():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    assert '"/planning/compute-path",' in source
    assert 'methods=["POST"]' in source


def test_application_requires_owned_planning():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        'def planning_compute_path():'
    )
    end = source.index(
        '@app.route("/planning/stop"',
        start,
    )
    route = source[start:end]

    assert "stop_robot()" in route
    assert "planning_control.snapshot()" in route
    assert "not planning.get('running')" in route
    assert "not planning.get('owned')" in route
    assert "publisher_node.compute_path(" in route


def test_no_motion_execution_capability_added():
    service = Path(
        "planning_path_service.py"
    ).read_text(encoding="utf-8")

    for marker in (
        "NavigateToPose",
        "FollowPath",
        "controller_server",
        "bt_navigator",
        "cmd_vel",
        "publish_motion",
    ):
        assert marker not in service
