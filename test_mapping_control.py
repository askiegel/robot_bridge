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
    """Legacy export-test placeholder; no runtime ownership."""
    def __init__(self, pid=4321):
        self.pid = pid


class FakeServiceCompleted:
    def __init__(self, returncode=0, stdout=''):
        self.returncode = returncode
        self.stdout = stdout


class FakeServiceRunner:
    def __init__(
        self,
        active=False,
        pid=4321,
        cgroup='/system.slice/mayday-slam.service',
    ):
        self.active = active
        self.pid = pid
        self.cgroup = cgroup
        self.calls = []
        self.actions = []

    def __call__(self, command, **kwargs):
        del kwargs
        command = [str(value) for value in command]
        self.calls.append(command)

        if (
            command[0] == '/usr/bin/systemctl'
            and command[1] == 'show'
        ):
            return FakeServiceCompleted(
                stdout=(
                    'ActiveState='
                    + ('active' if self.active else 'inactive')
                    + '\n'
                    + 'SubState='
                    + ('running' if self.active else 'dead')
                    + '\n'
                    + 'MainPID='
                    + (str(self.pid) if self.active else '0')
                    + '\n'
                    + 'ControlGroup='
                    + (self.cgroup if self.active else '')
                    + '\n'
                )
            )

        if command[:3] == [
            '/usr/bin/sudo',
            '-n',
            '/usr/bin/systemctl',
        ]:
            action = command[3]

            assert command[4] == 'mayday-slam.service'
            assert action in {'start', 'stop'}

            self.actions.append(action)

            if action == 'start':
                self.active = True
            else:
                self.active = False

            return FakeServiceCompleted()

        raise AssertionError(
            f'Unexpected service command: {command}'
        )


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


def make_control(
    tmp_path,
    active=False,
    process_finder=None,
    cgroup='/system.slice/mayday-slam.service',
):
    launch_path = tmp_path / 'slam.launch.py'
    launch_path.write_text(
        '# test launch\n',
        encoding='utf-8',
    )

    service = FakeServiceRunner(
        active=active,
        cgroup=cgroup,
    )

    control = MappingControl(
        launch_path=launch_path,
        log_path=tmp_path / 'mapping.log',
        service_runner=service,
        process_finder=(
            process_finder
            or (lambda owned_pid: [])
        ),
    )

    return control, service


def test_snapshot_adopts_active_systemd_service(tmp_path):
    control, service = make_control(
        tmp_path,
        active=True,
    )

    snapshot = control.snapshot()

    assert snapshot['running'] is True
    assert snapshot['owned'] is True
    assert snapshot['pid'] == service.pid
    assert snapshot['state'] == 'RUNNING'
    assert snapshot['systemd_service'] == (
        'mayday-slam.service'
    )
    assert snapshot['control_group'] == (
        '/system.slice/mayday-slam.service'
    )


def test_start_uses_protected_systemd_service(tmp_path):
    control, service = make_control(tmp_path)

    result = control.start('start')

    assert result['started'] is True
    assert result['pid'] == service.pid
    assert service.actions == ['start']

    assert control.command[1:4] == [
        'launch',
        'mini_pupper_slam',
        'slam.launch.py',
    ]


def test_duplicate_start_is_idempotent(tmp_path):
    control, service = make_control(tmp_path)

    first = control.start('first')
    second = control.start('second')

    assert first['started'] is True
    assert second['started'] is False
    assert second['action'] == 'ALREADY_RUNNING'
    assert second['pid'] == service.pid
    assert service.actions == ['start']


def test_external_process_is_rejected(tmp_path):
    control, service = make_control(
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

    assert service.actions == []


def test_missing_launch_is_rejected(tmp_path):
    service = FakeServiceRunner()

    control = MappingControl(
        launch_path=tmp_path / 'missing.launch.py',
        service_runner=service,
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

    assert service.actions == []


def test_owned_session_stops_systemd_service(tmp_path):
    control, service = make_control(
        tmp_path,
        active=True,
    )

    result = control.stop('stop')

    assert result['stopped'] is True
    assert result['state'] == 'STOPPED'
    assert result['stopped_pid'] == service.pid
    assert service.actions == ['stop']


def test_unexpected_cgroup_blocks_stop(tmp_path):
    control, service = make_control(
        tmp_path,
        active=True,
        cgroup='/system.slice/not-mayday.service',
    )

    try:
        control.stop('stop')
    except MappingControlError as error:
        assert 'expected systemd cgroup' in str(error)
    else:
        raise AssertionError(
            'Unexpected service ownership was stopped.'
        )

    assert service.active is True
    assert service.actions == []


def test_bridge_shutdown_does_not_stop_systemd_mapping(
    tmp_path,
):
    control, service = make_control(
        tmp_path,
        active=True,
    )

    control.shutdown()

    assert service.active is True
    assert service.actions == []
    assert control.snapshot()['running'] is True


def test_mapping_snapshot_has_no_save_or_motion(tmp_path):
    control, _ = make_control(tmp_path)
    snapshot = control.snapshot()

    assert snapshot['headless'] is True
    assert snapshot['planning_enabled'] is False
    assert snapshot['control_enabled'] is False
    assert snapshot['map_save_enabled'] is True
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

class FakeCompleted:
    def __init__(self, returncode=0, stdout=''):
        self.returncode = returncode
        self.stdout = stdout


def make_export_control(tmp_path, fail_operation=None):
    launch_path = tmp_path / 'slam.launch.py'
    launch_path.write_text(
        '# test launch\n',
        encoding='utf-8',
    )

    converter = tmp_path / 'converter'
    converter.write_text(
        '#!/bin/sh\n',
        encoding='utf-8',
    )
    converter.chmod(0o755)

    candidate_root = tmp_path / 'maps'
    candidate_root.mkdir()

    process = FakeProcess()
    service = FakeServiceRunner(
        active=False,
        pid=process.pid,
    )
    calls = []
    signals = service.actions

    def process_factory(command, **kwargs):
        del command
        del kwargs
        return process

    def command_runner(command, **kwargs):
        del kwargs
        calls.append(command)
        joined = ' '.join(command)

        if (
            fail_operation is not None
            and fail_operation in joined
        ):
            return FakeCompleted(
                returncode=1,
                stdout='forced failure',
            )

        if '/get_trajectory_states' in command:
            return FakeCompleted(
                stdout=(
                    'response: '
                    'StatusResponse(code=0, message=""), '
                    'trajectory_id=[0], '
                    'trajectory_state=[0]'
                ),
            )

        if '/submap_list' in command:
            return FakeCompleted(
                stdout=(
                    'header:\n'
                    '  frame_id: map\n'
                    'submap:\n'
                    '- trajectory_id: 0\n'
                    '  submap_index: 0\n'
                    '  submap_version: 180\n'
                    '- trajectory_id: 0\n'
                    '  submap_index: 1\n'
                    '  submap_version: 180\n'
                    '- trajectory_id: 0\n'
                    '  submap_index: 2\n'
                    '  submap_version: 140\n'
                ),
            )

        if '/finish_trajectory' in command:
            return FakeCompleted(
                stdout='StatusResponse(code=0, message="")',
            )

        if '/write_state' in command:
            import re

            match = re.search(
                r"filename: '([^']+)'",
                joined,
            )
            assert match is not None
            Path(match.group(1)).write_bytes(b'pbstream')
            return FakeCompleted(
                stdout='StatusResponse(code=0, message="")',
            )

        if command[0] == str(converter):
            pbstream_argument = next(
                value
                for value in command
                if value.startswith(
                    '-pbstream_filename='
                )
            )
            stem_argument = next(
                value
                for value in command
                if value.startswith('-map_filestem=')
            )

            assert Path(
                pbstream_argument.split('=', 1)[1]
            ).is_file()

            stem = Path(
                stem_argument.split('=', 1)[1]
            )
            stem.with_suffix('.yaml').write_text(
                'image: candidate.pgm\n'
                'resolution: 0.05\n',
                encoding='utf-8',
            )
            stem.with_suffix('.pgm').write_bytes(
                b'P5\n1 1\n255\n\xff'
            )
            return FakeCompleted(stdout='converted')

        raise AssertionError(
            f'Unexpected command: {command}'
        )

    control = MappingControl(
        launch_path=launch_path,
        log_path=tmp_path / 'mapping.log',
        candidate_root=candidate_root,
        converter_path=converter,
        command_runner=command_runner,
        service_runner=service,
        process_factory=process_factory,
        process_finder=lambda owned_pid: [],
        identity_checker=lambda pid: pid == process.pid,
        get_process_group=lambda pid: pid,
        signal_process_group=(
            lambda pgid, sig: signals.append(
                (pgid, sig)
            )
        ),
    )

    return (
        control,
        process,
        candidate_root,
        calls,
        signals,
    )


def test_candidate_export_creates_review_artifacts(
    tmp_path,
):
    import json

    (
        control,
        process,
        candidate_root,
        calls,
        signals,
    ) = make_export_control(tmp_path)

    control.start('2026-08-07T21:00:00+00:00')
    result = control.save_candidate(
        '2026-08-07T21:10:11+00:00'
    )

    candidate = (
        candidate_root
        / 'mayday_map_candidate_20260807'
        'T211011Z'
    )

    assert result['saved'] is True
    assert result['action'] == 'CANDIDATE_SAVED'
    assert result['running'] is False
    assert candidate.is_dir()
    assert not any(
        path.name.endswith('.partial')
        for path in candidate_root.iterdir()
    )

    pbstream = list(candidate.glob('*.pbstream'))
    yaml_files = list(candidate.glob('*.yaml'))
    images = list(candidate.glob('*.pgm'))

    assert len(pbstream) == 1
    assert len(yaml_files) == 1
    assert len(images) == 1
    assert (candidate / 'SHA256SUMS').is_file()

    yaml_source = yaml_files[0].read_text(
        encoding='utf-8',
    )

    assert (
        f'image: {candidate.name}.pgm'
        in yaml_source
    )
    assert '.partial' not in yaml_source
    assert str(candidate_root) not in yaml_source
    assert (
        candidate / f'{candidate.name}.pgm'
    ).is_file()

    metadata = json.loads(
        (
            candidate / 'CANDIDATE_METADATA.json'
        ).read_text(encoding='utf-8')
    )

    assert metadata['promoted'] is False
    assert metadata['validated_map_changed'] is False
    assert metadata['trajectory_id'] == 0
    assert (
        metadata['submap_readiness']
        ['submap_count']
        == 3
    )
    assert (
        metadata['submap_readiness']
        ['mature_submap_count']
        == 3
    )
    assert metadata['status'] == (
        'CANDIDATE_REVIEW_REQUIRED'
    )
    assert signals == [
        'start',
        'stop',
    ]

    joined_calls = [
        ' '.join(call)
        for call in calls
    ]

    assert any(
        '/get_trajectory_states' in call
        for call in joined_calls
    )
    assert any(
        '/finish_trajectory' in call
        for call in joined_calls
    )
    assert any(
        '/write_state' in call
        for call in joined_calls
    )
    assert any(
        '-pbstream_filename=' in call
        and '-map_filestem=' in call
        for call in joined_calls
    )


def test_export_requires_owned_running_session(tmp_path):
    control, _, _, _, _ = make_export_control(
        tmp_path
    )

    try:
        control.save_candidate(
            '2026-08-07T21:10:11+00:00'
        )
    except MappingControlError as error:
        assert 'No owned mapping session' in str(error)
    else:
        raise AssertionError(
            'Stopped mapping session was exported.'
        )


def test_export_requires_exactly_one_active_trajectory(
    tmp_path,
):
    control, _, _, _, _ = make_export_control(
        tmp_path
    )
    control.start('2026-08-07T21:00:00+00:00')

    control._command_runner = lambda command, **kwargs: (
        FakeCompleted(
            stdout=(
                'StatusResponse(code=0), '
                'trajectory_id=[0, 1], '
                'trajectory_state=[0, 0]'
            )
        )
    )

    try:
        control.save_candidate(
            '2026-08-07T21:10:11+00:00'
        )
    except MappingControlError as error:
        assert 'Exactly one active trajectory' in str(
            error
        )
    else:
        raise AssertionError(
            'Multiple active trajectories were accepted.'
        )


def test_failed_export_removes_partial_candidate(
    tmp_path,
):
    (
        control,
        _,
        candidate_root,
        _,
        _,
    ) = make_export_control(
        tmp_path,
        fail_operation='/finish_trajectory',
    )

    control.start('2026-08-07T21:00:00+00:00')

    try:
        control.save_candidate(
            '2026-08-07T21:10:11+00:00'
        )
    except MappingControlError as error:
        assert 'FinishTrajectory failed' in str(error)
    else:
        raise AssertionError(
            'Failed candidate export was accepted.'
        )

    assert list(candidate_root.iterdir()) == []
    assert control.snapshot()['running'] is False


def test_save_candidate_endpoint(monkeypatch):
    class FakeSaveControl(FakeMappingControl):
        def save_candidate(self, timestamp):
            del timestamp
            self.started = True
            self.stopped = True
            candidate = {
                'name': 'mayday_map_candidate_test',
                'directory': '/tmp/candidate',
                'status': 'CANDIDATE_REVIEW_REQUIRED',
                'promoted': False,
            }
            result = self.snapshot()
            result['saved'] = True
            result['action'] = 'CANDIDATE_SAVED'
            result['candidate'] = candidate
            return result

    control = FakeSaveControl()

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

    response = bridge.app.test_client().post(
        '/mapping/save-candidate'
    )
    payload = response.get_json()

    assert response.status_code == 201
    assert payload['ok'] is True
    assert payload['candidate']['promoted'] is False
    assert payload['mapping']['running'] is False


def test_save_candidate_is_post_only():
    client = bridge.app.test_client()

    assert client.get(
        '/mapping/save-candidate'
    ).status_code == 405

def test_immature_submaps_block_candidate_export(
    tmp_path,
):
    (
        control,
        _,
        candidate_root,
        calls,
        _,
    ) = make_export_control(tmp_path)

    original_runner = control._command_runner

    def immature_runner(command, **kwargs):
        if '/submap_list' in command:
            return FakeCompleted(
                stdout=(
                    'header:\n'
                    '  frame_id: map\n'
                    'submap:\n'
                    '- trajectory_id: 0\n'
                    '  submap_index: 0\n'
                    '  submap_version: 25\n'
                ),
            )

        return original_runner(command, **kwargs)

    control._command_runner = immature_runner
    control.start('2026-08-07T21:00:00+00:00')

    try:
        control.save_candidate(
            '2026-08-07T21:10:11+00:00'
        )
    except MappingControlError as error:
        assert 'not ready for candidate export' in str(
            error
        )
        assert '1 submaps' in str(error)
        assert '0 mature submaps' in str(error)
    else:
        raise AssertionError(
            'Immature submaps were exported.'
        )

    joined_calls = [
        ' '.join(call)
        for call in calls
    ]

    assert not any(
        '/finish_trajectory' in call
        for call in joined_calls
    )
    assert not any(
        '/write_state' in call
        for call in joined_calls
    )
    assert list(candidate_root.iterdir()) == []
    assert control.snapshot()['running'] is False
