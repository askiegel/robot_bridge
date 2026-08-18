#!/usr/bin/env python3

from pathlib import Path
from types import SimpleNamespace

import app as bridge

from navigation_goal_service import (
    NavigationGoalTimeoutError,
    NavigationGoalValidationError,
)


class FakeNavigationControl:
    def __init__(self, running=True, owned=True):
        self.running = running
        self.owned = owned

    def snapshot(self):
        return {
            "running": self.running,
            "owned": self.owned,
            "execution_enabled": True,
            "goal_submission_enabled": True,
            "maximum_goal_distance_meters": 0.50,
            "maximum_execution_seconds": 15.0,
        }


class FakePublisher:
    def __init__(self):
        self.payload = None
        self.cancel_count = 0
        self.result = {
            "status": "NAVIGATION_SUCCEEDED",
            "executed": True,
            "bounded": True,
        }
        self.error = None

    def execute_navigation_goal(self, payload):
        self.payload = payload

        if self.error is not None:
            raise self.error

        return self.result

    def cancel_navigation_goal(self):
        self.cancel_count += 1
        return {
            "active": True,
            "cancel_requested": True,
            "cancel_signal_sent": True,
        }


def configure(
    monkeypatch,
    *,
    running=True,
    owned=True,
):
    publisher = FakePublisher()
    stops = []

    monkeypatch.setattr(
        bridge,
        "navigation_control",
        FakeNavigationControl(
            running=running,
            owned=owned,
        ),
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

    def fake_stop():
        stops.append(True)
        return {
            "ok": True,
            "linear_x": 0.0,
            "angular_z": 0.0,
        }

    monkeypatch.setattr(
        bridge,
        "stop_robot",
        fake_stop,
    )

    return publisher, stops


def test_navigation_goal_route_is_post_only():
    client = bridge.app.test_client()

    response = client.get("/navigation/goal")

    assert response.status_code == 405


def test_owned_navigation_is_required(monkeypatch):
    _, stops = configure(
        monkeypatch,
        running=False,
        owned=False,
    )

    client = bridge.app.test_client()
    response = client.post(
        "/navigation/goal",
        json={
            "goal_x": 0.1,
            "goal_y": 0.0,
        },
    )

    assert response.status_code == 409
    assert response.get_json()["ok"] is False
    assert len(stops) == 1


def test_bounded_goal_success_has_two_safety_zeros(
    monkeypatch,
):
    publisher, stops = configure(monkeypatch)

    client = bridge.app.test_client()
    response = client.post(
        "/navigation/goal",
        json={
            "goal_x": 0.1,
            "goal_y": 0.0,
            "goal_yaw": 0.0,
        },
    )

    payload = response.get_json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["result"]["bounded"] is True
    assert publisher.payload == {
        "goal_x": 0.1,
        "goal_y": 0.0,
        "goal_yaw": 0.0,
    }
    assert len(stops) == 2


def test_validation_failure_has_final_safety_zero(
    monkeypatch,
):
    publisher, stops = configure(monkeypatch)
    publisher.error = NavigationGoalValidationError(
        "Localization pose is stale."
    )

    client = bridge.app.test_client()
    response = client.post(
        "/navigation/goal",
        json={"goal_x": 0.1, "goal_y": 0.0},
    )

    assert response.status_code == 400
    assert len(stops) == 2


def test_timeout_has_final_safety_zero(monkeypatch):
    publisher, stops = configure(monkeypatch)
    publisher.error = NavigationGoalTimeoutError(
        "Navigation exceeded the fixed "
        "15-second execution limit."
    )

    client = bridge.app.test_client()
    response = client.post(
        "/navigation/goal",
        json={"goal_x": 0.1, "goal_y": 0.0},
    )

    assert response.status_code == 504
    assert len(stops) == 2


def test_navigation_stop_cancels_before_lifecycle_stop():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        "def navigation_stop():"
    )
    end = source.index(
        '@app.route("/planning/status"',
        start,
    )
    route = source[start:end]

    assert (
        route.index("cancel_navigation_goal()")
        < route.index("stop_robot()")
        < route.index("navigation_control.stop(")
    )


def test_emergency_stop_cancels_navigation_goal():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index("def stop():")
    end = source.index("def main():", start)
    route = source[start:end]

    assert (
        route.index("cancel_navigation_goal()")
        < route.index("stop_robot()")
    )
    assert '"cancel_result": cancel_result' in route


def test_goal_service_uses_live_localization_snapshot():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        "def execute_navigation_goal"
    )
    end = source.index(
        "def cancel_navigation_goal",
        start,
    )
    method = source[start:end]

    assert "localization_telemetry.snapshot()" in method
    assert "navigation_goal_service.execute(" in method


def test_navigation_capabilities_are_enabled():
    source = Path(
        "navigation_control.py"
    ).read_text(encoding="utf-8")

    assert "'execution_enabled': True" in source
    assert "'goal_submission_enabled': True" in source
