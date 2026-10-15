"""Resolved STOP replies after retirement; real watchdog, simulated transport only."""
import copy
import threading

import pytest

from champ_heartbeat import HeartbeatCommands, STOP, EMERGENCY, START, HEARTBEAT
from test_champ_heartbeat import rig, gate
from test_continuous_lidar_safety import no_network


def retirement_race(rig, *, manager_class=None, pause_seconds=.4, stop_call=None):
    """Reproduce the retained ordering, with explicitly synthetic native timing.

    HTTP STOP is EMERGENCY. Both its HTTP waiter and the command's finish
    waiter observe the same packet. Finish commits exact zero and detaches
    the command before the HTTP waiter attempts to return its result.
    """
    g, node, adapter, manager = rig
    if manager_class is not None:
        manager = manager_class(adapter, 'host-1', clock=g.clock, sleep=g.clock.sleep,
            epoch='bridge-1', lidar_reader=manager.lidar_reader)
    parked = threading.Event(); resume_command = threading.Event()
    acknowledged = threading.Event(); resume_reply = threading.Event()
    result = {}; errors = []; records = {}; events = []
    original_ack = manager._ack
    original_sleep = manager.sleep
    def sleep(seconds):
        original_sleep(seconds)
        c = manager.current
        if (threading.current_thread() is command_thread and c and not parked.is_set()
                and g.clock() >= c['start'] + round(min(pause_seconds, .5) * 1e9)):
            parked.set()
            assert resume_command.wait(3.)
    manager.sleep = sleep
    def ack(packet, **kwargs):
        value = original_ack(packet, **kwargs)
        if threading.current_thread() is stop_thread:
            records.update(packet=copy.deepcopy(packet), ack=value,
                evidence=copy.deepcopy(adapter.snapshot()[0]), command=manager.current)
            events.append(('http_ack_validated', g.clock()))
            acknowledged.set()
            assert resume_reply.wait(3.)
        return value
    manager._ack = ack
    def command():
        try: result['command'] = manager.execute(0., 0., -.25, .5)
        except BaseException as error: errors.append(error)
    def stop():
        try: result['http_stop'] = stop_call() if stop_call is not None else manager.emergency_stop()
        except BaseException as error: errors.append(error)
    command_thread = threading.Thread(target=command)
    stop_thread = threading.Thread(target=stop)
    command_thread.start()
    try:
        assert parked.wait(3.)
        if pause_seconds > .5:
            g.clock.sleep(pause_seconds - .5)
            result['native_before_stop'] = adapter.snapshot()[0]
        stop_thread.start()
        assert acknowledged.wait(3.)
        assert records['ack'][0]
        resume_command.set()
        command_thread.join(3.)
        assert not command_thread.is_alive() and manager.current is None
        assert records['command']['stop_result'] == (True, 'emergency_gait_zero')
        result['committed_zero_before_http_reply'] = True
        result['stop_packet'] = records['packet']
        result['stop_evidence'] = records['evidence']
        events.append(('command_finished_with_zero', g.clock()))
    finally:
        resume_command.set(); resume_reply.set()
        command_thread.join(3.)
        if stop_thread.ident is not None: stop_thread.join(3.)
    assert not errors, errors
    assert not stop_thread.is_alive()
    result.update(events=events, packets=copy.deepcopy(node.messages), quarantined=manager.latched)
    return result


@pytest.mark.parametrize('pause_seconds', [.4, .5, .51])
def test_same_emergency_ack_after_command_detachment_is_not_owner_conflict(rig, pause_seconds):
    result = retirement_race(rig, pause_seconds=pause_seconds)
    assert result['http_stop']['ok'], result
    assert result['http_stop']['reason'] == 'emergency_gait_zero'
    assert result['http_stop']['controller_gait_zero']
    assert result['command']['state'] == 'interrupted'
    assert result['command']['reason'] == 'command_cancelled'
    assert result['command']['controller_gait_zero']
    assert not result['command']['completed_progress_credit']
    assert result['command']['stop_certificate'] is None  # Emergency is not scoped STOP.
    assert not result['quarantined']
    if pause_seconds > .5:
        assert result['native_before_stop']['reason'] == 'command_deadline_expired'
        assert result['native_before_stop']['active'] is False
    packet = result['stop_packet']; evidence = result['stop_evidence']
    assert packet['operation'] == EMERGENCY
    assert evidence['last_request_issued_ns'] == packet['issued_ns']
    assert evidence['last_request_generation'] == packet['generation']
    assert evidence['last_request_sequence'] == packet['sequence']
    assert evidence['active'] is False and evidence['gait_zero']
    i = result['packets'].index(packet)
    assert all(p['operation'] not in (START, HEARTBEAT) for p in result['packets'][i+1:])


@pytest.mark.parametrize('duration', [.1, .5])
def test_normal_right_turn_still_completes_only_after_immutable_deadline(rig, duration):
    g, node, adapter, manager = rig
    result = manager.execute(0., 0., -.25, duration)
    assert result['state'] == 'completed' and result['completed_progress_credit']
    assert result['reason'] == 'bounded_deadline_completed'
    assert result['stop_reason'] == 'scoped_gait_zero'
    record = manager.stop_request
    assert record['packet']['operation'] == STOP
    assert record['packet']['issued_ns'] >= result['deadline_ns']
    assert record['evidence']['last_request_issued_ns'] == record['packet']['issued_ns']
    count = len(node.messages)
    # An already committed outcome is not reversed by another waiter's timeout.
    assert manager._resolve_stop(record, (False, 'controller_ack_timeout')) == record['result']
    assert not manager.latched and manager.current is None and len(node.messages) == count
    assert not g.tick()['active']


@pytest.mark.parametrize('fault', ['current_successor', 'newer_stop', 'pending_newer_stop',
    'session', 'epoch', 'command_session', 'command_epoch', 'not_retired',
    'missing_result', 'command_result_mismatch', 'packet_mismatch', 'missing_evidence'])
def test_retired_response_cannot_cross_a_new_owner_or_invent_committed_proof(rig, fault):
    g, node, adapter, manager = rig
    assert manager.execute(0., 0., -.25, .5)['state'] == 'completed'
    record = manager.stop_request; command = record['command']
    if fault == 'current_successor': manager.current = {'generation': 2, 'retired': False}
    if fault in {'newer_stop', 'pending_newer_stop'}:
        g.clock.sleep(.001)
        with manager.lock:
            next_record = manager._begin_stop(manager._packet(EMERGENCY), None)
        if fault == 'newer_stop':
            manager._publish(next_record['packet'])
            assert manager._resolve_stop(next_record, manager._ack(next_record['packet'], zero=True,
                stop_request=next_record))[0]
    if fault == 'session': manager.session = 'other-controller'
    if fault == 'epoch': manager.epoch = 'other-bridge'
    if fault == 'command_session': command['controller_session'] = 'other-controller'
    if fault == 'command_epoch': command['bridge_epoch'] = 'other-bridge'
    if fault == 'not_retired': command['retired'] = False
    if fault == 'missing_result': record['result'] = None
    if fault == 'command_result_mismatch': command['stop_result'] = (False, 'unknown')
    if fault == 'packet_mismatch': command['stop_packet'] = dict(record['packet'], issued_ns=0)
    if fault == 'missing_evidence': record['evidence'] = None
    current, latest, latched = manager.current, manager.stop_request, manager.latched
    saved_record = copy.deepcopy(record)
    count = len(node.messages)
    assert manager._resolve_stop(record, (True, 'late_generic_zero')) == (False, 'retired_command_owner')
    assert manager.current is current and manager.stop_request is latest
    assert record == saved_record and manager.latched == latched
    assert len(node.messages) == count  # No publication, cancellation, latch or authority mutation.


@pytest.mark.parametrize('failure', ['missing_ack', 'delayed_ack', 'publication_uncertain'])
def test_unresolved_stop_after_retirement_remains_quarantined_despite_later_zero(rig, failure):
    g, node, adapter, manager = rig
    publish = node.publish
    def uncertain(msg):
        if msg.operation == STOP:
            if failure == 'missing_ack': return
            if failure == 'delayed_ack':
                g.clock.sleep(.251)
                return  # An eventual native zero is not a timely packet ACK.
            publish(msg)
            raise RuntimeError('Uncertain delivery')
        publish(msg)
    node.publish = uncertain
    result = manager.execute(0., 0., -.25, .5)
    assert result['state'] == 'uncertain' and manager.latched
    assert not result['completed_progress_credit']
    record = manager.stop_request
    assert record['result'][0] is False and manager.current is None
    # The real native deadline expired; this zero snapshot is not STOP proof.
    assert g.tick()['gait_zero'] and not g.last['active']
    count = len(node.messages)
    assert manager._resolve_stop(record, (True, 'unsubstantiated_zero')) == record['result']
    assert record['result'][0] is False and manager.latched
    assert not manager.execute(0., 0., -.25, .5)['transport_attempted']
    assert len(node.messages) == count


def test_old_scoped_packet_and_callback_cannot_revoke_successor(rig):
    g, node, adapter, manager = rig
    assert manager.execute(0., 0., -.25, .5)['state'] == 'completed'
    old = manager.stop_request; packet = copy.deepcopy(old['packet'])
    checked = []
    def successor_started(p):
        if p['operation'] == START:
            node.hook = None
            current = manager.current
            count = len(node.messages)
            assert manager._resolve_stop(old, (False, 'late_timeout')) == (False, 'retired_command_owner')
            manager._finish(old['command'], 'uncertain', 'old_completion', True)
            manager._stop(old['command'])  # Historical reply only; no STOP publication.
            assert manager.current is current and not manager.latched and len(node.messages) == count
            native = g.send(packet)
            assert not native['last_request_accepted'] and native['active']
            assert native['generation'] == current['generation']
            checked.append(True)
    node.hook = successor_started
    result = manager.execute(0., 0., -.25, .5)
    assert checked and result['state'] == 'completed' and result['completed_progress_credit']
    assert not manager.latched


def test_retired_stop_reply_through_real_flask_and_current_cognitive_client(rig, monkeypatch):
    import io
    import json
    import sys
    import urllib.error
    from types import SimpleNamespace
    import app as bridge  # main()/ROS initialization is never executed.
    monkeypatch.syspath_prepend('/home/tkieg/mayday_worktrees/cognitive-forward-fresh-scan-recovery')
    for name in list(sys.modules):
        if name == 'robot_bridge' or name.startswith('robot_bridge.'):
            monkeypatch.delitem(sys.modules, name)
    from robot_bridge.client import RobotBridgeClient
    manager = rig[3]
    monkeypatch.setattr(bridge, 'ros_ready', True)
    monkeypatch.setattr(bridge, 'publisher_node', SimpleNamespace(champ_commands=manager,
        cancel_navigation_goal=lambda: {'active': False, 'cancel_requested': False}))
    local = bridge.app.test_client(); statuses = []
    def urlopen(request, **kwargs):
        assert request.method == 'POST' and request.full_url == 'http://offline.invalid/stop'
        response = local.post('/stop', json={}); statuses.append(response.status_code)
        if response.status_code != 200:
            raise urllib.error.HTTPError(request.full_url, response.status_code,
                'simulated Flask', {}, io.BytesIO(response.data))
        return io.BytesIO(response.data)
    monkeypatch.setattr('urllib.request.urlopen', urlopen)
    client = RobotBridgeClient(base_url='http://offline.invalid')
    result = retirement_race(rig, stop_call=client.stop)
    assert statuses == [200] and result['http_stop']['ok']
    assert result['http_stop']['stop_result']['controller_gait_zero']
    assert result['http_stop']['stop_result']['reason'] == 'emergency_gait_zero'
    assert result['command']['state'] == 'interrupted'
    assert not result['command']['completed_progress_credit']
    assert not result['quarantined']
