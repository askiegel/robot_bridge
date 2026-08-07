#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import signal
import subprocess
from pathlib import Path

import app as bridge
from mapping_control import (
    MappingConflictError,
    MappingControl,
    MappingControlError,
)


class FakeProcess:
    def __init__(self, pid=4321, exited=False):
        self.pid = pid
        self.returncode = 1 if exited else None
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.wait_calls.append(timeout)
        self.returncode = 0
        return 0


class FakeMappingControl:
    def __init__(self):
        self.started = False
        self.stopped = False

    def snapshot(self):
        running = self.started and not self.stopped

        return {
            'running': running,
            'owned': running,
            'pid': 4321 if running else None,
            'state': 'RUNNING' if running else 'STOPPED',
            'planning_enabled': False,
            'control_enabled': False,
            'map_save_enabled': False,
            'validated_map_mutable': False,
        }

    def start(self, timestamp):
        del timestamp
        already_running = self.started and not self.stopped
        self.started = True
        self.stopped = False

        result = self.snapshot()
        result['started'] = not already_running
        result['action'] = (
            'ALREADY_RUNNING'
            if already_running
            else 'STARTED'
        )
        return result

    def stop(self, timestamp):
        del timestamp
        was_running = self.started and not self.stopped
        self.stopped = True

        result = self.snapshot()
        result['stopped'] = was_running
        result['action'] = (
            'STOPPED'
            if was_running
            else 'ALREADY_STOPPED'
        )
        return result


def make_control(tmp_path, process_finder=None):
    launch_path = tmp_path / 'slam.launch.py'
    launch_path.write_text(
        '# test launch\n',
        encoding='utf-8',
    )

    process = FakeProcess()
    calls = []
    signals = []

    def process_factory(command, **kwargs):
        calls.append((command, kwargs))
        return process

    control = MappingControl(
        launch_path=launch_path,
        log_path=tmp_path / 'mapping.log',
        process_factory=process_factory,
        process_finder=(
            process_finder
            or (lambda owned_pid: [])
        ),
        identity_checker=lambda pid: pid == process.pid,
        get_process_group=lambda pid: pid,
        signal_process_group=(
            lambda pgid, sig: signals.append(
                (pgid, sig)
            )
        ),
    )

    return control, process, calls, signals


def test_start_uses_fixed_headless_mapping_command(tmp_path):
    control, process, calls, _ = make_control(tmp_path)

    result = control.start('start')
    command, kwargs = calls[0]

    assert result['started'] is True
    assert result['pid'] == process.pid
    assert command[1:4] == [
        'launch',
        'mini_pupper_slam',
        'slam.launch.py',
    ]
    assert 'use_sim_time:=false' in command
    assert 'use_rviz:=false' in command
    assert kwargs['start_new_session'] is True
    assert kwargs['stdin'] is subprocess.DEVNULL

    joined = ' '.join(command)
    assert 'planner_server' not in joined
    assert 'controller_server' not in joined
    assert 'map_saver' not in joined


def test_duplicate_start_is_idempotent(tmp_path):
    control, process, calls, _ = make_control(tmp_path)

    first = control.start('first')
    second = control.start('second')

    assert first['started'] is True
    assert second['started'] is False
    assert second['action'] == 'ALREADY_RUNNING'
    assert second['pid'] == process.pid
    assert len(calls) == 1


def test_external_process_is_rejected(tmp_path):
    control, _, calls, _ = make_control(
        tmp_path,
        process_finder=lambda owned_pid: [999],
    )

    try:
        control.start('start')
    except MappingConflictError as error:
        assert '999' in str(error)
    else:
        raise AssertionError(
            'External process was not rejected.'
        )

    assert calls == []


def test_missing_launch_is_rejected(tmp_path):
    control = MappingControl(
        launch_path=tmp_path / 'missing.launch.py',
        process_finder=lambda owned_pid: [],
    )

    try:
        control.start('start')
    except MappingControlError as error:
        assert 'launch is unavailable' in str(error)
    else:
        raise AssertionError(
            'Missing launch was accepted.'
        )


def test_owned_session_stops_verified_group(tmp_path):
    control, process, _, signals = make_control(tmp_path)

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
    control, process, _, signals = make_control(tmp_path)
    control.start('start')
    control._identity_checker = lambda pid: False

    try:
        control.stop('stop')
    except MappingControlError as error:
        assert 'identity changed' in str(error)
    else:
        raise AssertionError(
            'Changed PID identity was signalled.'
        )

    assert process.poll() is None
    assert signals == []


def test_mapping_snapshot_has_no_save_or_motion(tmp_path):
    control, _, _, _ = make_control(tmp_path)
    snapshot = control.snapshot()

    assert snapshot['headless'] is True
    assert snapshot['planning_enabled'] is False
    assert snapshot['control_enabled'] is False
    assert snapshot['map_save_enabled'] is False
    assert snapshot['validated_map_mutable'] is False


def test_start_endpoint_publishes_zero(monkeypatch):
    control = FakeMappingControl()

    monkeypatch.setattr(
        bridge,
        'mapping_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {
            'ok': True,
            'linear_x': 0.0,
            'angular_z': 0.0,
        },
    )
    monkeypatch.setattr(
        bridge.localization_control,
        'snapshot',
        lambda: {'running': False},
    )

    response = bridge.app.test_client().post(
        '/mapping/start'
    )
    payload = response.get_json()

    assert response.status_code == 201
    assert payload['ok'] is True
    assert payload['mapping']['running'] is True
    assert payload['stop_result']['linear_x'] == 0.0
    assert payload['stop_result']['angular_z'] == 0.0


def test_start_is_blocked_when_localization_runs(
    monkeypatch,
):
    control = FakeMappingControl()

    monkeypatch.setattr(
        bridge,
        'mapping_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )
    monkeypatch.setattr(
        bridge.localization_control,
        'snapshot',
        lambda: {'running': True},
    )

    response = bridge.app.test_client().post(
        '/mapping/start'
    )

    assert response.status_code == 409
    assert control.started is False


def test_stop_endpoint_is_idempotent(monkeypatch):
    control = FakeMappingControl()

    monkeypatch.setattr(
        bridge,
        'mapping_control',
        control,
    )
    monkeypatch.setattr(
        bridge,
        'stop_robot',
        lambda: {'ok': True},
    )

    first = bridge.app.test_client().post(
        '/mapping/stop'
    )
    second = bridge.app.test_client().post(
        '/mapping/stop'
    )

    assert first.status_code == 200
    assert second.status_code == 200
    assert first.get_json()['mapping']['stopped'] is False
    assert second.get_json()['mapping']['stopped'] is False


def test_status_and_method_scope(monkeypatch):
    control = FakeMappingControl()

    monkeypatch.setattr(
        bridge,
        'mapping_control',
        control,
    )

    client = bridge.app.test_client()

    assert client.get('/mapping/status').status_code == 200
    assert client.post('/mapping/status').status_code == 405
    assert client.get('/mapping/start').status_code == 405
    assert client.get('/mapping/stop').status_code == 405
