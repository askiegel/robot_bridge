#!/usr/bin/env python3

from pathlib import Path

import app as bridge

from navigation_goal_service import (
    NavigationGoalValidationError,
)


def stopped_state():
    return {
        "running": False,
        "owned": False,
        "state": "STOPPED",
    }


def running_mapping():
    return {
        "running": True,
        "owned": True,
        "pid": 59303,
        "state": "RUNNING",
    }


def running_mapping_navigation():
    return {
        "running": True,
        "owned": True,
        "pid": 4321,
        "state": "RUNNING",
        "mode": "live_mapping",
        "mapping_ready": True,
        "execution_enabled": True,
        "goal_submission_enabled": True,
        "maximum_goal_distance_meters": 0.50,
        "maximum_execution_seconds": 25.0,
    }


def fresh_mapping_pose():
    return {
        "available": True,
        "status": "READY",
        "source": "cartographer_tf",
        "age_seconds": 0.05,
        "error": None,
        "pose": {
            "frame_id": "map",
            "source_frame_id": "base_link",
            "position": {
                "x": 0.60,
                "y": 0.10,
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0,
            },
        },
    }


class StaticControl:
    def __init__(self, state):
        self.state = dict(state)

    def snapshot(self):
        return dict(self.state)


class FakeMappingNavigationControl:
    def __init__(self, state=None, events=None):
        self.state = dict(
            state or stopped_state()
        )
        self.events = events if events is not None else []
        self.start_calls = []
        self.stop_calls = []

    def snapshot(self):
        return dict(self.state)

    def start(self, timestamp):
        self.events.append("lifecycle_start")
        self.start_calls.append(timestamp)

        self.state = running_mapping_navigation()

        result = dict(self.state)
        result["action"] = "STARTED"
        result["started"] = True

        return result

    def stop(self, timestamp):
        self.events.append("lifecycle_stop")
        self.stop_calls.append(timestamp)

        old_pid = self.state.get("pid")

        self.state = stopped_state()

        result = dict(self.state)
        result["action"] = "STOPPED"
        result["stopped"] = True
        result["stopped_pid"] = old_pid

        return result


class FakePublisher:
    def __init__(
        self,
        pose=None,
        preflight=None,
        goal_result=None,
        goal_error=None,
    ):
        self.pose = (
            pose
            if pose is not None
            else fresh_mapping_pose()
        )
        self.preflight = (
            preflight
            if preflight is not None
            else {
                "ok": True,
                "failures": [],
            }
        )
        self.goal_result = (
            goal_result
            if goal_result is not None
            else {
                "status": "NAVIGATION_SUCCEEDED",
                "executed": True,
                "bounded": True,
                "maximum_goal_distance_meters": 0.50,
                "maximum_execution_seconds": 25.0,
            }
        )
        self.goal_error = goal_error

        self.pose_calls = 0
        self.preflight_calls = 0
        self.goal_payload = None

    def navigation_start_preflight(self):
        self.preflight_calls += 1
        return dict(self.preflight)

    def mapping_pose_snapshot(self):
        self.pose_calls += 1
        return dict(self.pose)

    def execute_mapping_navigation_goal(
        self,
        payload,
    ):
        self.goal_payload = payload

        if self.goal_error is not None:
            raise self.goal_error

        return dict(self.goal_result)


def install_common_start_state(
    monkeypatch,
    mapping_navigation=None,
    publisher=None,
):
    if mapping_navigation is None:
        mapping_navigation = (
            FakeMappingNavigationControl()
        )

    if publisher is None:
        publisher = FakePublisher()

    monkeypatch.setattr(
        bridge,
        "mapping_control",
        StaticControl(running_mapping()),
    )
    monkeypatch.setattr(
        bridge,
        "navigation_control",
        StaticControl(stopped_state()),
    )
    monkeypatch.setattr(
        bridge,
        "planning_control",
        StaticControl(stopped_state()),
    )
    monkeypatch.setattr(
        bridge,
        "localization_control",
        StaticControl(stopped_state()),
    )
    monkeypatch.setattr(
        bridge,
        "mapping_navigation_control",
        mapping_navigation,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        publisher,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        True,
    )
    monkeypatch.setattr(
        bridge,
        "ros_error",
        None,
    )
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: {
            "ok": True,
            "linear_x": 0.0,
            "angular_z": 0.0,
        },
    )

    return mapping_navigation, publisher


def test_mapping_navigation_status_is_read_only(
    monkeypatch,
):
    control = FakeMappingNavigationControl(
        running_mapping_navigation()
    )

    monkeypatch.setattr(
        bridge,
        "mapping_navigation_control",
        control,
    )

    client = bridge.app.test_client()

    response = client.get(
        "/mapping-navigation/status"
    )

    payload = response.get_json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert (
        payload["mapping_navigation"]["mode"]
        == "live_mapping"
    )

    assert client.post(
        "/mapping-navigation/status"
    ).status_code == 405


def test_mapping_navigation_start_requires_mapping(
    monkeypatch,
):
    control, _ = install_common_start_state(
        monkeypatch
    )

    monkeypatch.setattr(
        bridge,
        "mapping_control",
        StaticControl(stopped_state()),
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/start"
    )

    assert response.status_code == 409
    assert control.start_calls == []


def test_mapping_navigation_start_rejects_saved_nav(
    monkeypatch,
):
    control, _ = install_common_start_state(
        monkeypatch
    )

    monkeypatch.setattr(
        bridge,
        "navigation_control",
        StaticControl({
            "running": True,
            "owned": True,
        }),
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/start"
    )

    assert response.status_code == 409
    assert control.start_calls == []


def test_mapping_navigation_start_requires_fresh_pose(
    monkeypatch,
):
    stale = fresh_mapping_pose()
    stale["age_seconds"] = 10.0

    control, publisher = install_common_start_state(
        monkeypatch,
        publisher=FakePublisher(
            pose=stale
        ),
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/start"
    )

    payload = response.get_json()

    assert response.status_code == 503
    assert control.start_calls == []
    assert publisher.preflight_calls == 1
    assert publisher.pose_calls == 1
    assert "pose is not ready" in payload["error"]


def test_mapping_navigation_start_does_not_stop_mapping(
    monkeypatch,
):
    control, publisher = install_common_start_state(
        monkeypatch
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/start"
    )

    payload = response.get_json()

    assert response.status_code == 201
    assert payload["ok"] is True
    assert publisher.preflight_calls == 1
    assert publisher.pose_calls == 1
    assert len(control.start_calls) == 1

    assert (
        payload["mapping"]["running"]
        is True
    )
    assert (
        payload["mapping"]["owned"]
        is True
    )


def test_mapping_navigation_goal_uses_mapping_path(
    monkeypatch,
):
    stop_calls = []

    control = FakeMappingNavigationControl(
        running_mapping_navigation()
    )
    publisher = FakePublisher()

    monkeypatch.setattr(
        bridge,
        "mapping_navigation_control",
        control,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        publisher,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        True,
    )
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: (
            stop_calls.append("zero")
            or {
                "ok": True,
                "linear_x": 0.0,
                "angular_z": 0.0,
            }
        ),
    )

    request_payload = {
        "goal_x": 0.75,
        "goal_y": 0.10,
        "goal_yaw": 0.0,
    }

    response = bridge.app.test_client().post(
        "/mapping-navigation/goal",
        json=request_payload,
    )

    payload = response.get_json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert (
        publisher.goal_payload
        == request_payload
    )
    assert stop_calls == [
        "zero",
        "zero",
    ]


def test_mapping_navigation_goal_validation_failure_stops(
    monkeypatch,
):
    stop_calls = []

    control = FakeMappingNavigationControl(
        running_mapping_navigation()
    )
    publisher = FakePublisher(
        goal_error=NavigationGoalValidationError(
            "Goal exceeds the fixed 0.50-meter limit."
        )
    )

    monkeypatch.setattr(
        bridge,
        "mapping_navigation_control",
        control,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        publisher,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        True,
    )
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: (
            stop_calls.append("zero")
            or {"ok": True}
        ),
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/goal",
        json={
            "goal_x": 5.0,
            "goal_y": 0.0,
        },
    )

    assert response.status_code == 400
    assert stop_calls == [
        "zero",
        "zero",
    ]


def test_mapping_navigation_stop_cancels_first(
    monkeypatch,
):
    events = []

    control = FakeMappingNavigationControl(
        running_mapping_navigation(),
        events=events,
    )

    monkeypatch.setattr(
        bridge,
        "mapping_navigation_control",
        control,
    )

    monkeypatch.setattr(
        bridge,
        "cancel_navigation_goal",
        lambda: (
            events.append("cancel")
            or {
                "active": True,
                "cancel_requested": True,
                "cancel_signal_sent": True,
            }
        ),
    )

    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: (
            events.append("zero")
            or {"ok": True}
        ),
    )

    response = bridge.app.test_client().post(
        "/mapping-navigation/stop"
    )

    assert response.status_code == 200
    assert events == [
        "cancel",
        "zero",
        "lifecycle_stop",
    ]


def test_mapping_navigation_actions_are_post_only():
    client = bridge.app.test_client()

    assert client.get(
        "/mapping-navigation/start"
    ).status_code == 405

    assert client.get(
        "/mapping-navigation/goal"
    ).status_code == 405

    assert client.get(
        "/mapping-navigation/stop"
    ).status_code == 405


def test_mapping_goal_uses_same_guarded_service():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        "def execute_mapping_navigation_goal"
    )
    end = source.index(
        "def execute_navigation_goal",
        start,
    )

    method = source[start:end]

    assert "self.mapping_pose_snapshot()" in method
    assert "self.navigation_goal_service.execute(" in method
    assert "localization_telemetry" not in method

    saved_start = source.index(
        "def execute_navigation_goal"
    )
    saved_end = source.index(
        "def cancel_navigation_goal",
        saved_start,
    )

    saved_method = source[
        saved_start:saved_end
    ]

    assert (
        "self.refresh_planning_localization_pose()"
        in saved_method
    )
    assert (
        "localization_telemetry.snapshot()"
        in saved_method
    )
