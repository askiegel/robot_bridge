#!/usr/bin/env python3

import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

import app as bridge
from planning_localization_initializer import (
    PlanningLocalizationConflictError,
    PlanningLocalizationInitializer,
    PlanningLocalizationTimeoutError,
    PlanningLocalizationUnavailableError,
)


@pytest.fixture(autouse=True)
def disable_initializer_sleep(monkeypatch):
    monkeypatch.setattr(
        PlanningLocalizationInitializer,
        'NOMOTION_UPDATE_INTERVAL_SECONDS',
        0.0,
    )


class ImmediateFuture:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        if self._error is not None:
            raise self._error

        return self._result


class PendingFuture:
    def add_done_callback(self, callback):
        self._callback = callback


class FakeClient:
    def __init__(
        self,
        available=True,
        future_factory=None,
    ):
        self.available = available
        self.future_factory = (
            future_factory
            or (
                lambda: ImmediateFuture(
                    result=SimpleNamespace()
                )
            )
        )
        self.requests = []
        self.wait_timeouts = []

    def wait_for_service(self, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.future_factory()


class FakeNode:
    def __init__(
        self,
        global_client=None,
        nomotion_client=None,
    ):
        self.global_client = (
            global_client or FakeClient()
        )
        self.nomotion_client = (
            nomotion_client or FakeClient()
        )
        self.service_names = []

    def create_client(self, service_type, name):
        self.service_names.append(name)

        if name == (
            PlanningLocalizationInitializer
            .GLOBAL_SERVICE
        ):
            return self.global_client

        if name == (
            PlanningLocalizationInitializer
            .NOMOTION_SERVICE
        ):
            return self.nomotion_client

        raise AssertionError(name)


def running_planning():
    return {
        'running': True,
        'owned': True,
        'planning_enabled': True,
        'controller_enabled': False,
        'navigator_enabled': False,
        'execution_enabled': False,
        'motion_enabled': False,
    }


def stopped_planning():
    return {
        'running': False,
        'owned': False,
        'planning_enabled': True,
        'controller_enabled': False,
        'navigator_enabled': False,
        'execution_enabled': False,
        'motion_enabled': False,
    }


def test_fixed_amcl_service_names():
    node = FakeNode()

    PlanningLocalizationInitializer(node)

    assert node.service_names == [
        '/reinitialize_global_localization',
        '/request_nomotion_update',
    ]


def test_initializer_uses_only_empty_requests():
    node = FakeNode()
    initializer = PlanningLocalizationInitializer(
        node
    )

    result = initializer.initialize()

    assert len(node.global_client.requests) == 1
    assert (
        len(node.nomotion_client.requests)
        == initializer.NOMOTION_UPDATE_COUNT
    )
    assert (
        result['nomotion_updates_requested']
        == initializer.NOMOTION_UPDATE_COUNT
    )
    assert result['global_localization_requested']
    assert result['stationary_required']
    assert result['pose_published'] is False
    assert result['initial_pose_supplied'] is False


def test_launch_pose_is_cleared_before_nomotion_updates():
    events = []

    class RecordingClient(FakeClient):
        def __init__(self, label):
            super().__init__()
            self.label = label

        def call_async(self, request):
            events.append(self.label)
            return super().call_async(request)

    global_client = RecordingClient("global")
    nomotion_client = RecordingClient("nomotion")
    node = FakeNode(
        global_client=global_client,
        nomotion_client=nomotion_client,
    )

    initializer = PlanningLocalizationInitializer(
        node,
        pose_clearer=lambda: events.append(
            "clear_pose"
        ),
    )

    initializer.initialize()

    assert events[0:2] == [
        "global",
        "clear_pose",
    ]
    assert events[-2:] == [
        "clear_pose",
        "nomotion",
    ]
    assert events.count("global") == 1
    assert events.count("clear_pose") == 2
    assert (
        events.count("nomotion")
        == initializer.NOMOTION_UPDATE_COUNT
    )


def test_pose_clearer_runs_once_per_initialization():
    clear_count = []

    initializer = PlanningLocalizationInitializer(
        FakeNode(),
        pose_clearer=lambda: clear_count.append(1),
    )

    initializer.initialize()

    assert clear_count == [1, 1]


def test_convergence_sequence_is_bounded():
    sleeps = []

    initializer = PlanningLocalizationInitializer(
        FakeNode(),
        sleeper=lambda seconds: sleeps.append(
            seconds
        ),
    )

    initializer.initialize()

    assert (
        len(sleeps)
        == initializer.NOMOTION_UPDATE_COUNT - 1
    )
    assert all(
        seconds
        == initializer.NOMOTION_UPDATE_INTERVAL_SECONDS
        for seconds in sleeps
    )


def test_final_update_follows_second_cache_clear():
    events = []

    class RecordingClient(FakeClient):
        def __init__(self, label):
            super().__init__()
            self.label = label

        def call_async(self, request):
            events.append(self.label)
            return super().call_async(request)

    initializer = PlanningLocalizationInitializer(
        FakeNode(
            global_client=RecordingClient(
                "global"
            ),
            nomotion_client=RecordingClient(
                "nomotion"
            ),
        ),
        pose_clearer=lambda: events.append(
            "clear_pose"
        ),
        sleeper=lambda seconds: None,
    )

    initializer.initialize()

    assert events[-2:] == [
        "clear_pose",
        "nomotion",
    ]


def test_result_explicitly_disables_execution():
    initializer = PlanningLocalizationInitializer(
        FakeNode()
    )

    result = initializer.initialize()

    assert result['path_computed'] is False
    assert result['path_executed'] is False
    assert (
        result['navigation_goal_executed']
        is False
    )
    assert result['controller_enabled'] is False
    assert result['navigator_enabled'] is False
    assert result['motion_enabled'] is False


def test_missing_global_service_is_blocked():
    node = FakeNode(
        global_client=FakeClient(
            available=False
        )
    )
    initializer = PlanningLocalizationInitializer(
        node
    )

    with pytest.raises(
        PlanningLocalizationUnavailableError,
        match="reinitialize_global_localization",
    ):
        initializer.initialize()

    assert not node.global_client.requests
    assert not node.nomotion_client.requests


def test_missing_nomotion_service_is_blocked():
    node = FakeNode(
        nomotion_client=FakeClient(
            available=False
        )
    )
    initializer = PlanningLocalizationInitializer(
        node
    )

    with pytest.raises(
        PlanningLocalizationUnavailableError,
        match="request_nomotion_update",
    ):
        initializer.initialize()

    assert not node.global_client.requests
    assert not node.nomotion_client.requests


def test_service_timeout_is_bounded(monkeypatch):
    initializer = PlanningLocalizationInitializer(
        FakeNode(
            global_client=FakeClient(
                future_factory=PendingFuture
            )
        )
    )

    monkeypatch.setattr(
        initializer,
        'RESPONSE_TIMEOUT_SECONDS',
        0.01,
    )

    with pytest.raises(
        PlanningLocalizationTimeoutError,
        match="timed out",
    ):
        initializer.initialize()


def test_concurrent_initialization_is_rejected():
    initializer = PlanningLocalizationInitializer(
        FakeNode()
    )

    assert initializer._request_lock.acquire(
        blocking=False
    )

    try:
        with pytest.raises(
            PlanningLocalizationConflictError,
            match="already active",
        ):
            initializer.initialize()
    finally:
        initializer._request_lock.release()


def test_route_is_post_only():
    client = bridge.app.test_client()

    response = client.get(
        '/planning/initialize-localization'
    )

    assert response.status_code == 405


def test_route_requires_owned_planning(monkeypatch):
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )
    monkeypatch.setattr(
        bridge.planning_control,
        'snapshot',
        stopped_planning,
    )

    response = bridge.app.test_client().post(
        '/planning/initialize-localization',
        json={},
    )

    assert response.status_code == 409
    assert response.get_json()['ok'] is False


def test_route_rejects_request_parameters(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )
    monkeypatch.setattr(
        bridge.planning_control,
        'snapshot',
        running_planning,
    )
    monkeypatch.setattr(
        bridge,
        'ros_ready',
        True,
    )
    monkeypatch.setattr(
        bridge,
        'publisher_node',
        SimpleNamespace(),
    )

    response = bridge.app.test_client().post(
        '/planning/initialize-localization',
        json={'x': 1.0},
    )

    assert response.status_code == 400
    assert response.get_json()['ok'] is False


def test_route_returns_stationary_result(
    monkeypatch,
):
    result = {
        'global_localization_requested': True,
        'nomotion_updates_requested': 20,
        'stationary_required': True,
        'path_computed': False,
        'path_executed': False,
        'navigation_goal_executed': False,
        'controller_enabled': False,
        'navigator_enabled': False,
        'motion_enabled': False,
    }

    fake_node = SimpleNamespace(
        initialize_planning_localization=(
            lambda: dict(result)
        )
    )

    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )
    monkeypatch.setattr(
        bridge.planning_control,
        'snapshot',
        running_planning,
    )
    monkeypatch.setattr(
        bridge.localization_telemetry,
        'clear',
        lambda: None,
    )
    monkeypatch.setattr(
        bridge,
        'ros_ready',
        True,
    )
    monkeypatch.setattr(
        bridge,
        'publisher_node',
        fake_node,
    )

    response = bridge.app.test_client().post(
        '/planning/initialize-localization',
        json={},
    )

    assert response.status_code == 200

    payload = response.get_json()

    assert payload['ok'] is True
    assert payload['initialization'] == result


def test_source_has_no_motion_capability():
    initializer_source = Path(
        'planning_localization_initializer.py'
    ).read_text(encoding='utf-8')
    app_source = Path('app.py').read_text(
        encoding='utf-8'
    )

    forbidden_initializer = (
        'NavigateToPose',
        'cmd_vel',
        'Twist',
        'create_publisher',
        'ActionClient',
        'controller_server',
        'bt_navigator',
        'goal_x',
        'goal_y',
    )

    for marker in forbidden_initializer:
        assert marker not in initializer_source

    route_start = app_source.index(
        '@app.route(\n'
        '    "/planning/initialize-localization"'
    )
    route_end = app_source.index(
        '@app.route(\n'
        '    "/planning/compute-path"',
        route_start,
    )
    route_source = app_source[
        route_start:route_end
    ]

    for marker in (
        'goal_x',
        'goal_y',
        'NavigateToPose',
        'controller_server',
        'bt_navigator',
    ):
        assert marker not in route_source
