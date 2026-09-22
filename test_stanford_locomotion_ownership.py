#!/usr/bin/env python3

import subprocess

import app as bridge
from navigation_goal_service import NavigationGoalValidationError


def test_ownership_status_inactive(monkeypatch):
    class Completed:
        returncode = 0
        stdout = "ActiveState=inactive\nSubState=dead\nStatusText=\nResult=success\nExecMainStatus=0\n"
        stderr = ""

    calls = []
    monkeypatch.setattr(subprocess, "run", lambda command, **kwargs: calls.append(command) or Completed())
    result = bridge.stanford_ownership_status()
    assert result["ok"] is True
    assert result["ready"] is False
    assert result["active_state"] == "inactive"
    assert calls[0][:4] == ["systemctl", "show", bridge.STANFORD_LOCOMOTION_UNIT, "-p"]


def test_ownership_status_active_ready(monkeypatch):
    class Completed:
        returncode = 0
        stdout = "ActiveState=active\nSubState=running\nStatusText=" + bridge.STANFORD_OWNERSHIP_READY_STATUS + "\nResult=success\nExecMainStatus=0\n"
        stderr = ""

    monkeypatch.setattr(subprocess, "run", lambda command, **kwargs: Completed())
    result = bridge.stanford_ownership_status()
    assert result["ok"] is True
    assert result["ready"] is True


def test_systemctl_wrapper_is_bounded_and_exact(monkeypatch):
    class Completed:
        returncode = 0
        stdout = "started"
        stderr = ""

    calls = []
    monkeypatch.setattr(subprocess, "run", lambda command, **kwargs: calls.append((command, kwargs)) or Completed())
    result = bridge._run_stanford_systemctl("start", 12.5)
    assert result == {"ok": True, "returncode": 0, "stdout": "started", "stderr": ""}
    assert calls == [(["sudo", "-n", "systemctl", "start", bridge.STANFORD_LOCOMOTION_UNIT], {"capture_output": True, "text": True, "timeout": 12.5, "check": False})]


def test_acquisition_starts_inactive_owner(monkeypatch):
    states = iter([{"ok": True, "ready": False, "active_state": "inactive"}, {"ok": True, "ready": True, "active_state": "active"}])
    calls = []
    monkeypatch.setattr(bridge, "stanford_ownership_status", lambda: next(states))
    monkeypatch.setattr(bridge, "_run_stanford_systemctl", lambda action, timeout: calls.append((action, timeout)) or {"ok": True})
    result = bridge.acquire_stanford_ownership()
    assert result["ok"] is True
    assert result["started"] is True
    assert calls == [("start", bridge.STANFORD_OWNERSHIP_START_TIMEOUT_SECONDS)]


def test_acquisition_failure_reports_cleanup(monkeypatch):
    states = iter([
        {"ok": True, "ready": False, "active_state": "inactive"},
        {"ok": True, "ready": False, "active_state": "failed"},
        {"ok": True, "ready": True, "active_state": "active"},
        {"ok": True, "ready": False, "active_state": "inactive"},
    ])
    calls = []
    monkeypatch.setattr(bridge, "stanford_ownership_status", lambda: next(states))
    monkeypatch.setattr(bridge, "_run_stanford_systemctl", lambda action, timeout: calls.append(action) or {"ok": action == "stop"})
    result = bridge.acquire_stanford_ownership()
    assert result["ok"] is False
    assert result["cleanup"]["ok"] is True
    assert calls == ["start", "stop"]


def test_release_success_and_failure(monkeypatch):
    states = iter([{"ok": True, "active_state": "active"}, {"ok": True, "active_state": "inactive"}])
    calls = []
    monkeypatch.setattr(bridge, "stanford_ownership_status", lambda: next(states))
    monkeypatch.setattr(bridge, "_run_stanford_systemctl", lambda action, timeout: calls.append(action) or {"ok": True})
    result = bridge.release_stanford_ownership()
    assert result["ok"] is True
    assert calls == ["stop"]
    monkeypatch.setattr(bridge, "stanford_ownership_status", lambda: {"ok": True, "active_state": "active"})
    monkeypatch.setattr(bridge, "_run_stanford_systemctl", lambda action, timeout: {"ok": False})
    failed = bridge.release_stanford_ownership()
    assert failed["ok"] is False
    assert failed["error"]


class FakeNavigationControl:
    def snapshot(self):
        return {"running": True, "owned": True, "execution_enabled": True, "goal_submission_enabled": True}

    def stop(self, timestamp):
        return {"stopped": True, "timestamp": timestamp}


class FakePublisher:
    def __init__(self, error=None):
        self.error = error
        self.calls = []

    def execute_navigation_goal(self, payload):
        self.calls.append(payload)
        if self.error:
            raise self.error
        return {"status": "NAVIGATION_SUCCEEDED", "bounded": True}

    def cancel_navigation_goal(self):
        return {"active": False, "cancel_requested": False}


def configure_route(monkeypatch, publisher=None):
    publisher = publisher or FakePublisher()
    stops = []
    releases = []
    monkeypatch.setattr(bridge, "navigation_control", FakeNavigationControl())
    monkeypatch.setattr(bridge, "publisher_node", publisher)
    monkeypatch.setattr(bridge, "ros_ready", True)
    monkeypatch.setattr(bridge, "ros_error", None)
    monkeypatch.setattr(bridge, "stop_robot", lambda: stops.append(True) or {"ok": True, "linear_x": 0.0, "angular_z": 0.0})
    monkeypatch.setattr(bridge, "release_stanford_ownership", lambda: releases.append(True) or {"ok": True, "stopped": True})
    return publisher, stops, releases


def test_navigation_goal_acquisition_failure_prevents_execution(monkeypatch):
    publisher, stops, releases = configure_route(monkeypatch)
    monkeypatch.setattr(bridge, "acquire_stanford_ownership", lambda: {"ok": False, "error": "ownership unavailable"})
    response = bridge.app.test_client().post("/navigation/goal", json={"goal_x": 0.1, "goal_y": 0.0})
    payload = response.get_json()
    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["stanford_ownership"]["error"] == "ownership unavailable"
    assert publisher.calls == []
    assert releases == []
    assert len(stops) == 2


def test_navigation_goal_failure_releases_owner(monkeypatch):
    publisher, stops, releases = configure_route(monkeypatch, FakePublisher(NavigationGoalValidationError("invalid goal")))
    ownership = {"ok": True, "started": True, "reused": False}
    monkeypatch.setattr(bridge, "acquire_stanford_ownership", lambda: ownership)
    response = bridge.app.test_client().post("/navigation/goal", json={"goal_x": 0.1, "goal_y": 0.0})
    payload = response.get_json()
    assert response.status_code == 400
    assert payload["stanford_ownership"] == ownership
    assert releases == [True]
    assert len(stops) == 2


def test_navigation_goal_success_preserves_owner_diagnostics(monkeypatch):
    publisher, stops, releases = configure_route(monkeypatch)
    ownership = {"ok": True, "started": False, "reused": True}
    after = {"ok": True, "ready": True, "active_state": "active"}
    monkeypatch.setattr(bridge, "acquire_stanford_ownership", lambda: ownership)
    monkeypatch.setattr(bridge, "stanford_ownership_status", lambda: after)
    response = bridge.app.test_client().post("/navigation/goal", json={"goal_x": 0.1, "goal_y": 0.0})
    payload = response.get_json()
    assert response.status_code == 200
    assert payload["stanford_ownership"] == ownership
    assert payload["stanford_ownership_after"] == after
    assert publisher.calls
    assert releases == []
    assert len(stops) == 2


def test_navigation_stop_preserves_release_diagnostics(monkeypatch):
    stops = []
    releases = []
    monkeypatch.setattr(bridge, "cancel_navigation_goal", lambda: {"active": True, "cancel_requested": True})
    monkeypatch.setattr(bridge, "stop_robot", lambda: stops.append(True) or {"ok": True, "linear_x": 0.0, "angular_z": 0.0})
    monkeypatch.setattr(bridge, "release_stanford_ownership", lambda: releases.append(True) or {"ok": True, "stopped": True})
    monkeypatch.setattr(bridge, "navigation_control", FakeNavigationControl())
    monkeypatch.setattr(bridge.localization_telemetry, "clear", lambda: None)
    response = bridge.app.test_client().post("/navigation/stop")
    payload = response.get_json()
    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["stanford_ownership_release"]["ok"] is True
    assert releases == [True]
    assert len(stops) == 1
