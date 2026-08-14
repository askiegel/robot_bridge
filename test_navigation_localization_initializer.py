# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

from pathlib import Path
from types import SimpleNamespace

import pytest

import app as bridge


ROUTE = "/navigation/initialize-localization"


def stopped_navigation():
    return {
        "running": False,
        "owned": False,
        "state": "STOPPED",
    }


def owned_navigation():
    return {
        "running": True,
        "owned": True,
        "state": "RUNNING",
        "execution_enabled": True,
        "goal_submission_enabled": True,
    }


def safety_zero():
    return {
        "ok": True,
        "linear_x": 0.0,
        "angular_z": 0.0,
    }


def stationary_result():
    return {
        "action": (
            "PLANNING_LOCALIZATION_INITIALIZED"
        ),
        "global_localization_requested": True,
        "nomotion_updates_requested": 20,
        "stationary_required": True,
        "pose_published": False,
        "initial_pose_supplied": False,
        "path_computed": False,
        "path_executed": False,
        "navigation_goal_executed": False,
        "controller_enabled": False,
        "navigator_enabled": False,
        "motion_enabled": False,
    }


def prepare_owned_navigation(
    monkeypatch,
    node=None,
    stop_results=None,
):
    if node is None:
        node = SimpleNamespace(
            initialize_planning_localization=(
                stationary_result
            ),
        )

    if stop_results is None:
        stop_results = [
            safety_zero(),
            safety_zero(),
        ]

    results = iter(stop_results)

    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: next(results),
    )
    monkeypatch.setattr(
        bridge.navigation_control,
        "snapshot",
        owned_navigation,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        True,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        node,
    )

    return node


def assert_zero(result):
    assert result["ok"] is True
    assert result["linear_x"] == 0.0
    assert result["angular_z"] == 0.0


def test_route_is_post_only():
    response = bridge.app.test_client().get(
        ROUTE
    )

    assert response.status_code == 405


def test_route_requires_owned_navigation(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        safety_zero,
    )
    monkeypatch.setattr(
        bridge.navigation_control,
        "snapshot",
        stopped_navigation,
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == 409
    assert payload["ok"] is False
    assert (
        "owned guarded navigation"
        in payload["error"].lower()
    )
    assert_zero(payload["initial_stop_result"])


def test_route_does_not_accept_planning_ownership(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        safety_zero,
    )
    monkeypatch.setattr(
        bridge.navigation_control,
        "snapshot",
        stopped_navigation,
    )
    monkeypatch.setattr(
        bridge.planning_control,
        "snapshot",
        lambda: {
            "running": True,
            "owned": True,
        },
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )

    assert response.status_code == 409


def test_route_rejects_request_parameters(
    monkeypatch,
):
    node = prepare_owned_navigation(
        monkeypatch
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={"retries": 1},
    )
    payload = response.get_json()

    assert response.status_code == 400
    assert payload["ok"] is False
    assert "does not accept" in payload["error"]
    assert_zero(payload["initial_stop_result"])
    assert_zero(payload["final_stop_result"])

    assert (
        node.initialize_planning_localization
        is not None
    )


def test_route_requires_ros(
    monkeypatch,
):
    results = iter([
        safety_zero(),
        safety_zero(),
    ])

    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: next(results),
    )
    monkeypatch.setattr(
        bridge.navigation_control,
        "snapshot",
        owned_navigation,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        False,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        None,
    )
    monkeypatch.setattr(
        bridge,
        "ros_error",
        "ROS unavailable",
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["error"] == "ROS unavailable"
    assert_zero(payload["initial_stop_result"])
    assert_zero(payload["final_stop_result"])


def test_route_aborts_when_initial_zero_fails(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge,
        "stop_robot",
        lambda: {
            "ok": False,
            "error": "zero failed",
        },
    )
    monkeypatch.setattr(
        bridge.navigation_control,
        "snapshot",
        owned_navigation,
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == 503
    assert payload["ok"] is False
    assert (
        "safety zero"
        in payload["error"].lower()
    )


def test_route_returns_stationary_result(
    monkeypatch,
):
    calls = []

    class FakeNode:
        def initialize_planning_localization(
            self,
        ):
            calls.append("initialize")
            return stationary_result()

    prepare_owned_navigation(
        monkeypatch,
        node=FakeNode(),
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert calls == ["initialize"]
    assert (
        payload["action"]
        == "navigation_initialize_localization"
    )
    assert (
        payload["initialization"]
        == stationary_result()
    )
    assert_zero(payload["initial_stop_result"])
    assert_zero(payload["final_stop_result"])
    assert payload["navigation"]["owned"] is True


def test_route_reports_final_zero_failure(
    monkeypatch,
):
    prepare_owned_navigation(
        monkeypatch,
        stop_results=[
            safety_zero(),
            {
                "ok": False,
                "error": "final zero failed",
            },
        ],
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == 503
    assert payload["ok"] is False
    assert (
        "final safety zero"
        in payload["error"].lower()
    )
    assert (
        payload["initialization"]
        == stationary_result()
    )


@pytest.mark.parametrize(
    ("exception_type", "expected_status"),
    (
        (
            bridge.PlanningLocalizationConflictError,
            409,
        ),
        (
            bridge.PlanningLocalizationUnavailableError,
            503,
        ),
        (
            bridge.PlanningLocalizationTimeoutError,
            504,
        ),
        (
            bridge.PlanningLocalizationError,
            422,
        ),
    ),
)
def test_route_maps_initializer_errors(
    monkeypatch,
    exception_type,
    expected_status,
):
    class FakeNode:
        def initialize_planning_localization(
            self,
        ):
            raise exception_type(
                "initializer failed"
            )

    prepare_owned_navigation(
        monkeypatch,
        node=FakeNode(),
    )

    response = bridge.app.test_client().post(
        ROUTE,
        json={},
    )
    payload = response.get_json()

    assert response.status_code == expected_status
    assert payload["ok"] is False
    assert payload["error"] == "initializer failed"
    assert_zero(payload["initial_stop_result"])
    assert_zero(payload["final_stop_result"])


def test_existing_planning_guard_is_preserved():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        '@app.route(\n'
        '    "/planning/initialize-localization"'
    )
    end = source.index(
        '@app.route(\n'
        '    "/planning/compute-path"',
        start,
    )

    planning_route = source[start:end]

    assert (
        "Owned planning runtime is not active."
        in planning_route
    )
    assert "navigation_control" not in planning_route


def test_navigation_route_has_no_goal_capability():
    source = Path("app.py").read_text(
        encoding="utf-8"
    )

    start = source.index(
        '@app.route(\n'
        '    "/navigation/initialize-localization"'
    )
    end = source.index(
        '@app.route("/navigation/goal"',
        start,
    )

    route = source[start:end]

    for forbidden in (
        "execute_navigation_goal",
        "compute_path",
        "goal_x",
        "goal_y",
        "NavigateToPose",
        "send_goal",
        "send_goal_async",
    ):
        assert forbidden not in route

    assert (
        ".initialize_planning_localization()"
        in route
    )
