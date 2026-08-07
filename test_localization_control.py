#!/usr/bin/env python3
"""Tests for guarded localization-only process control."""

import hashlib
import signal
import subprocess
from pathlib import Path

import app as bridge
from localization_control import (
    LocalizationConflictError,
    LocalizationControl,
    LocalizationControlError,
)


class FakeProcess:
    def __init__(self, pid=4321):
        self.pid = pid
        self.return_code = None
        self.wait_calls = []

    def poll(self):
        return self.return_code

    def wait(self, timeout):
        self.wait_calls.append(timeout)
        self.return_code = 0
        return 0


class FakeControl:
    def __init__(self):
        self.started = False
        self.stopped = False

    def snapshot(self):
        return {
            'running': self.started and not self.stopped,
            'owned': self.started and not self.stopped,
            'pid': 4321 if self.started else None,
            'state': (
                'RUNNING'
                if self.started and not self.stopped
                else 'STOPPED'
            ),
            'planning_enabled': False,
            'control_enabled': False,
        }

    def start(self, timestamp):
        del timestamp
        self.started = True
        result = self.snapshot()
        result['started'] = True
        result['action'] = 'STARTED'
        return result

    def stop(self, timestamp):
        del timestamp
        self.stopped = True
        result = self.snapshot()
        result['stopped'] = True
        result['action'] = 'STOPPED'
        return result


def create_map(tmp_path):
    names = (
        'mayday_supervised_route_03.pbstream',
        'mayday_supervised_route_03.yaml',
        'mayday_supervised_route_03.pgm',
    )
    lines = []

    for name in names:
        path = tmp_path / name
        path.write_bytes(name.encode('utf-8'))
        digest = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        lines.append(f'{digest}  {path}')

    (tmp_path / 'SHA256SUMS').write_text(
        '\n'.join(lines) + '\n',
        encoding='utf-8',
    )


def make_control(tmp_path, **overrides):
    create_map(tmp_path)

    process = overrides.pop(
        'process',
        FakeProcess(),
    )
    calls = []

    def factory(command, **kwargs):
        calls.append((command, kwargs))
        return process

    control = LocalizationControl(
        map_directory=tmp_path,
        log_path=tmp_path / 'localization.log',
        process_factory=overrides.pop(
            'process_factory',
            factory,
        ),
        process_finder=overrides.pop(
            'process_finder',
            lambda owned_pid: [],
        ),
        identity_checker=overrides.pop(
            'identity_checker',
            lambda pid: True,
        ),
        get_process_group=overrides.pop(
            'get_process_group',
            lambda pid: pid,
        ),
        signal_process_group=overrides.pop(
            'signal_process_group',
            lambda group, requested_signal: None,
        ),
        **overrides,
    )

    control.ROS2_EXECUTABLE = (
        Path('/opt/ros/humble/bin/ros2')
    )

    return control, process, calls


def test_start_uses_fixed_localization_only_command(tmp_path):
    control, process, calls = make_control(tmp_path)

    result = control.start('start-time')

    assert result['started'] is True
    assert result['running'] is True
    assert result['pid'] == process.pid
    assert len(calls) == 1

    command, kwargs = calls[0]

    assert command[0] == '/opt/ros/humble/bin/ros2'
    assert command[1:4] == [
        'launch',
        'mini_pupper_navigation',
        'localization.launch.py',
    ]
    assert 'autostart:=true' in command
    assert kwargs['start_new_session'] is True
    assert kwargs['stdin'] is subprocess.DEVNULL
    assert 'planner_server' not in ' '.join(command)
    assert 'controller_server' not in ' '.join(command)


def test_duplicate_start_is_idempotent(tmp_path):
    control, process, calls = make_control(tmp_path)

    first = control.start('first')
    second = control.start('second')

    assert first['started'] is True
    assert second['started'] is False
    assert second['action'] == 'ALREADY_RUNNING'
    assert second['pid'] == process.pid
    assert len(calls) == 1


def test_external_localization_is_rejected(tmp_path):
    control, _, _ = make_control(
        tmp_path,
        process_finder=lambda owned_pid: [999],
    )

    try:
        control.start('start')
    except LocalizationConflictError as exc:
        assert '999' in str(exc)
    else:
        raise AssertionError(
            'External localization was not rejected.'
        )


def test_bad_checksum_is_rejected(tmp_path):
    control, _, calls = make_control(tmp_path)

    (
        tmp_path
        / 'mayday_supervised_route_03.yaml'
    ).write_text('changed', encoding='utf-8')

    try:
        control.start('start')
    except LocalizationControlError as exc:
        assert 'checksum failed' in str(exc)
    else:
        raise AssertionError(
            'Invalid map checksum was accepted.'
        )

    assert calls == []


def test_owned_session_stops_verified_group(tmp_path):
    signals = []

    control, process, _ = make_control(
        tmp_path,
        signal_process_group=(
            lambda group, requested_signal:
            signals.append(
                (group, requested_signal)
            )
        ),
    )

    control.start('start')
    result = control.stop('stop')

    assert result['stopped'] is True
    assert result['state'] == 'STOPPED'
    assert result['stopped_pid'] == process.pid
    assert signals == [
        (process.pid, signal.SIGINT),
    ]
    assert process.wait_calls == [12.0]


def test_identity_change_blocks_signal(tmp_path):
    signals = []

    control, _, _ = make_control(
        tmp_path,
        identity_checker=lambda pid: False,
        signal_process_group=(
            lambda group, requested_signal:
            signals.append(
                (group, requested_signal)
            )
        ),
    )

    control.start('start')

    try:
        control.stop('stop')
    except LocalizationControlError as exc:
        assert 'identity changed' in str(exc)
    else:
        raise AssertionError(
            'Changed identity was signaled.'
        )

    assert signals == []


def test_start_endpoint_publishes_zero(monkeypatch):
    control = FakeControl()
    stop_calls = []

    monkeypatch.setattr(
        bridge,
        'localization_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: (
            stop_calls.append(True)
            or {
                'ok': True,
                'linear_x': 0.0,
                'angular_z': 0.0,
            }
        ),
    )

    client = bridge.app.test_client()
    response = client.post('/localization/start')
    payload = response.get_json()

    assert response.status_code == 201
    assert payload['ok'] is True
    assert payload['localization']['started'] is True
    assert stop_calls == [True]


def test_start_is_blocked_when_zero_fails(monkeypatch):
    control = FakeControl()

    monkeypatch.setattr(
        bridge,
        'localization_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': False},
    )

    client = bridge.app.test_client()
    response = client.post('/localization/start')

    assert response.status_code == 503
    assert control.started is False


def test_stop_endpoint_clears_pose(monkeypatch):
    control = FakeControl()
    control.started = True

    class FakeTelemetry:
        def __init__(self):
            self.cleared = False

        def clear(self):
            self.cleared = True

    telemetry = FakeTelemetry()

    monkeypatch.setattr(
        bridge,
        'localization_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'localization_telemetry',
        telemetry,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )

    client = bridge.app.test_client()
    response = client.post('/localization/stop')

    assert response.status_code == 200
    assert control.stopped is True
    assert telemetry.cleared is True


def test_status_is_get_only_and_controls_are_post_only(
    monkeypatch,
):
    control = FakeControl()

    monkeypatch.setattr(
        bridge,
        'localization_control',
        control,
    )

    client = bridge.app.test_client()

    assert client.get(
        '/localization/status'
    ).status_code == 200
    assert client.post(
        '/localization/status'
    ).status_code == 405
    assert client.get(
        '/localization/start'
    ).status_code == 405
    assert client.get(
        '/localization/stop'
    ).status_code == 405
