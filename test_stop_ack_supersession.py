"""Real native watchdog and ROS serialization; simulated transport, no DDS/network."""
import threading

import pytest

from champ_heartbeat import START, HEARTBEAT, STOP, EMERGENCY, TIMEOUT_NS
from test_champ_heartbeat import gate, rig
from test_continuous_lidar_safety import no_network


def assert_cancelled(result, manager, node):
    assert result['state'] == 'interrupted'
    assert result['controller_gait_zero']
    assert result['stop_reason'] == 'emergency_gait_zero'
    assert not result['completed_progress_credit']
    assert not result['physical_stop_verified']
    assert result['stop_certificate'] is None  # Never invent a scoped certificate.
    assert not manager.latched
    stop_at = next(i for i, p in enumerate(node.messages) if p['operation'] == EMERGENCY)
    assert all(p['operation'] not in (START, HEARTBEAT) for p in node.messages[stop_at+1:])


def test_emergency_ack_before_old_scoped_waiter_cannot_be_overwritten(rig):
    g, n, a, m = rig
    snapshot = a.snapshot
    cancelled = []
    def pending_application():
        state, receipt = snapshot()
        if state['last_operation'] == STOP and not cancelled:
            # The real controller first publishes acceptance with gait-zero
            # pending; its following control iteration applies zero.
            state['gait_zero'] = False
        return state, receipt
    def cancel(seconds):
        if m.current and m.current['retired'] and not cancelled:
            cancelled.append(True)
            g.clock.sleep(.001)
            assert m.emergency_stop()['ok']
        g.clock.sleep(seconds)
    a.snapshot = pending_application
    m.sleep = cancel
    before = g.clock()
    result = m.execute(.1, 0., 0., .1)
    assert_cancelled(result, m, n)
    assert g.clock() - before < 150_000_000  # No spurious 250 ms ACK tail.
    assert not g.tick()['active']


@pytest.mark.parametrize('old_operation', [STOP, EMERGENCY])
@pytest.mark.parametrize('old_timeout_first', [False, True])
def test_real_threads_old_waiter_cannot_reverse_new_verified_stop(rig, old_operation, old_timeout_first):
    g, n, a, m = rig
    held = threading.Event()
    release = threading.Event()
    answers = {}
    original = m._ack
    old_request = []
    def reordered(p, **kw):
        if p['operation'] == old_operation and not old_request:
            old_request.append(kw['stop_request'])
            if old_timeout_first:
                # The old response is absent; its *actual* ACK loop times out.
                snapshot = a.snapshot
                a.snapshot = lambda: (None, 0)
                try:
                    failed = original(p, **kw)
                finally:
                    a.snapshot = snapshot
                assert failed == (False, 'controller_ack_timeout')
            held.set()
            assert release.wait(3.)
            return failed if old_timeout_first else original(p, **kw)
        return original(p, **kw)
    m._ack = reordered
    if old_operation == EMERGENCY:
        cancelled = []
        def cancel(seconds):
            g.clock.sleep(seconds)
            if m.current and not cancelled:
                cancelled.append(True)
                answers['old_emergency'] = m.emergency_stop()
        m.sleep = cancel
    runner = threading.Thread(target=lambda: answers.update(command=m.execute(.1, 0., 0., .1)))
    runner.start()
    try:
        assert held.wait(3.)
        g.clock.sleep(.001)
        answers['new_emergency'] = m.emergency_stop()
        assert answers['new_emergency']['ok']
        winner = m.stop_request
        assert winner is not old_request[0]
    finally:
        release.set()
        runner.join(3.)
    assert not runner.is_alive()
    assert_cancelled(answers['command'], m, n)
    assert winner['result'] == (True, 'emergency_gait_zero')
    assert winner['evidence']['last_request_issued_ns'] == winner['packet']['issued_ns']
    if old_operation == EMERGENCY:
        assert answers['old_emergency']['ok']


def test_failed_stop_already_quarantined_is_not_automatically_rearmed(rig):
    g, n, a, m = rig
    publish = n.publish
    def drop(msg):
        if msg.operation != STOP:
            publish(msg)
    n.publish = drop
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert not result['controller_gait_zero']
    n.publish = publish
    assert m.emergency_stop()['ok']
    assert m.latched
    count = len(n.messages)
    assert not m.execute(.1, 0., 0., .1)['transport_attempted']
    assert len(n.messages) == count


@pytest.mark.parametrize('operation', [STOP, EMERGENCY])
@pytest.mark.parametrize('fault', [
    'controller_session', 'host_boot_id', 'bridge_epoch', 'generation', 'sequence',
    'deadline_ns', 'last_operation', 'last_request_generation',
    'last_request_sequence', 'last_request_issued_ns', 'last_request_accepted',
    'active', 'gait_zero', 'linear_x', 'linear_y', 'angular_z',
    'stale_native', 'stale_receipt', 'missing_identity', 'predates_packet',
])
def test_stop_needs_exact_current_native_identity_and_applied_zero(rig, operation, fault):
    g, n, a, m = rig
    if operation == EMERGENCY:
        def cancel(p):
            if p['operation'] == START:
                n.hook = None
                assert m.emergency_stop()['ok']
        n.hook = cancel
    result = m.execute(.1, 0., 0., .1)
    assert result['controller_gait_zero']
    p = next(p for p in reversed(n.messages) if p['operation'] == operation)
    state, receipt = a.snapshot()
    if fault == 'missing_identity':
        del state['last_request_issued_ns']
    elif fault == 'stale_native':
        state['evaluated_ns'] = g.clock() - TIMEOUT_NS
    elif fault == 'stale_receipt':
        receipt = g.clock() - TIMEOUT_NS
    elif fault == 'predates_packet':
        state['evaluated_ns'] = p['issued_ns'] - 1
    elif fault in ('active', 'gait_zero', 'last_request_accepted'):
        state[fault] = not state[fault]
    elif fault in ('linear_x', 'linear_y', 'angular_z'):
        state[fault] = .01
    else:
        value = state[fault]
        state[fault] = value + 1 if type(value) is int else 'different-owner'
    a.snapshot = lambda: (dict(state), receipt)
    assert not m._ack(p, zero=True)[0]


def test_new_pending_stop_cannot_reuse_old_verified_result_or_certificate(rig):
    g, n, a, m = rig
    original = m._ack
    def pending(p, **kw):
        if p['operation'] == STOP:
            assert original(p, **kw)[0]
            record = kw['stop_request']
            assert m._resolve_stop(record, (True, 'scoped_gait_zero'))[0]
            assert m.current['stop_certificate']
            def dropped(msg):
                if msg.operation != EMERGENCY:
                    raise AssertionError('No replacement movement permitted')
            n.publish = dropped
            g.clock.sleep(.001)
            assert not m.emergency_stop()['ok']
            assert m.current['stop_result'][0] is False
            assert m.current['stop_certificate'] is None
            return False, 'controller_ack_timeout'
        return original(p, **kw)
    m._ack = pending
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert not result['controller_gait_zero'] and result['stop_certificate'] is None


@pytest.mark.parametrize('first_ok', [True, False])
def test_second_waiter_for_same_packet_cannot_reverse_committed_outcome(rig, first_ok):
    g, n, a, m = rig
    assert m.execute(.1, 0., 0., .1)['state'] == 'completed'
    record = m.stop_request
    # Global STOP has no command owner; no command is admitted while pending.
    with m.lock:
        record = m._begin_stop(m._packet(EMERGENCY), None)
    if first_ok:
        m._publish(record['packet'])
        first = m._ack(record['packet'], zero=True, stop_request=record)
    else:
        first = (False, 'controller_ack_timeout')
    assert m._resolve_stop(record, first) == first
    assert m._resolve_stop(record, (not first_ok, 'late_waiter')) == first
    assert m.latched is not first_ok


def test_old_waiter_cannot_mutate_replacement_command_or_quarantine(rig):
    g, n, a, m = rig
    assert m.execute(.1, 0., 0., .1)['state'] == 'completed'
    old = m.stop_request
    def late(p):
        if p['operation'] == START:
            n.hook = None
            owner = m.current
            count = len(n.messages)
            assert not m._resolve_stop(old, (False, 'late_timeout'))[0]
            m._finish(old['command'], 'uncertain', 'old_completion', True)
            assert m.current is owner and not m.latched
            assert len(n.messages) == count
    n.hook = late
    assert m.execute(0., .08, 0., .1)['state'] == 'completed'


def test_pending_global_stop_blocks_admission_without_transport(rig):
    g, n, a, m = rig
    def pending(p):
        if p['operation'] == EMERGENCY:
            before = len(n.messages)
            result = m.execute(.1, 0., 0., .1)
            assert result['state'] == 'refused' and not result['transport_attempted']
            assert len(n.messages) == before
    # Establish the epoch before the global STOP, using simulated zero only.
    assert m._synchronize()[0]
    n.hook = pending
    assert m.emergency_stop()['ok']


@pytest.mark.parametrize('native_epoch', ['', 'previous-bridge'])
def test_global_emergency_before_sync_verifies_zero_without_acquiring_authority(rig, native_epoch):
    g, n, a, m = rig
    if native_epoch:
        assert g.sync(bridge_epoch=native_epoch)['accepted']
    assert m.emergency_stop()['ok'] and not m.latched
    state, _ = a.snapshot()
    assert state['bridge_epoch'] == native_epoch
    assert state['generation'] == 0 and state['sequence'] == 0
    assert not state['active'] and state['gait_zero']
    assert [p['operation'] for p in n.messages] == [EMERGENCY]


def test_global_emergency_rejects_controller_owner_change_even_with_exact_zero(rig):
    g, n, a, m = rig
    assert m._synchronize()[0]
    snapshot = a.snapshot
    def change_owner(p):
        if p['operation'] == EMERGENCY:
            n.hook = None
            state, receipt = snapshot()
            state['bridge_epoch'] = 'competing-bridge'
            a.snapshot = lambda: (dict(state), receipt)
    n.hook = change_owner
    assert not m.emergency_stop()['ok'] and m.latched


@pytest.mark.parametrize('field', ['bridge_epoch', 'generation', 'sequence', 'deadline_ns'])
def test_missing_global_owner_evidence_is_fail_closed(rig, field):
    g, n, a, m = rig
    snapshot = a.snapshot
    def missing():
        state, receipt = snapshot()
        del state[field]
        return state, receipt
    a.snapshot = missing
    assert not m.emergency_stop()['ok'] and m.latched


def test_positive_tuple_without_native_zero_evidence_cannot_resolve_stop(rig):
    g, n, a, m = rig
    assert m._synchronize()[0]
    with m.lock:
        request = m._begin_stop(m._packet(EMERGENCY), None)
    assert m._resolve_stop(request, (True, 'unsubstantiated_zero')) == (False, 'stop_evidence_missing')
    assert m.latched


def test_superseded_unresolved_stop_is_bounded_and_fail_closed(rig):
    g, n, a, m = rig
    original = m._ack
    def abandon_new(p, **kw):
        if p['operation'] == STOP:
            with m.lock:
                g.clock.sleep(.001)
                m._begin_stop(m._packet(EMERGENCY, m.current), m.current)
            before = g.clock()
            result = original(p, **kw)
            assert g.clock() - before <= TIMEOUT_NS
            return result
        return original(p, **kw)
    m._ack = abandon_new
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert not result['controller_gait_zero'] and not result['completed_progress_credit']


def test_missing_status_cannot_retain_an_earlier_successful_stop(rig):
    g, n, a, m = rig
    assert m.execute(.1, 0., 0., .1)['controller_gait_zero']
    a.snapshot = lambda: (None, 0)
    legacy = []
    a.emergency_zero = lambda: legacy.append(True)
    assert not m.emergency_stop()['ok']
    assert legacy and m.latched and not m.stop_request['result'][0]


def test_late_zero_after_request_deadline_cannot_resolve_failed_delivery(rig):
    g, n, a, m = rig
    assert m._synchronize()[0]
    with m.lock:
        record = m._begin_stop(m._packet(EMERGENCY), None)
        m._publish(record['packet'])
    g.clock.sleep(.25)
    # Fresh zero exists, but the STOP request's ACK deadline has elapsed.
    result = m._ack(record['packet'], zero=True, stop_request=record)
    assert result == (False, 'controller_ack_timeout')
    assert not m._resolve_stop(record, result)[0] and m.latched


def test_identical_packet_replay_is_not_an_independent_stop(rig):
    g, n, a, m = rig
    assert m._synchronize()[0]
    assert m.emergency_stop()['ok']
    first = dict(m.stop_request['packet'])
    # Frozen clock, same operation and command identity: indistinguishable ACK.
    assert not m.emergency_stop()['ok']
    assert m.stop_request['packet'] == first and m.latched


def test_command_publication_uncertainty_survives_verified_emergency_stop(rig):
    g, n, a, m = rig
    def cancel_then_fail(p):
        if p['operation'] == START:
            n.hook = None
            assert m.emergency_stop()['ok']
    n.hook = cancel_then_fail
    n.fail = lambda p: p['operation'] == START
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert result['controller_gait_zero'] and not result['completed_progress_credit']
    assert not m.execute(.1, 0., 0., .1)['transport_attempted']


@pytest.mark.parametrize('verified', [False, True])
def test_completion_rechecks_stop_started_after_old_resolution(rig, verified):
    g, n, a, m = rig
    original = m._stop
    def supersede_after_resolution(c):
        old = original(c)
        assert old[0]
        g.clock.sleep(.001)
        if verified:
            assert m.emergency_stop()['ok']
        else:
            with m.lock:
                c['cancelled'] = True
                m._begin_stop(m._packet(EMERGENCY, c), c)
        return old
    m._stop = supersede_after_resolution
    result = m.execute(.1, 0., 0., .1)
    if verified:
        assert_cancelled(result, m, n)
    else:
        assert result['state'] == 'uncertain' and m.latched
        assert not result['controller_gait_zero']


def test_controller_restart_during_emergency_cannot_certify_old_command(rig):
    g, n, a, m = rig
    original = m._ack
    def restart(p, **kw):
        if p['operation'] == START:
            state, receipt = a.snapshot()
            replacement = dict(state, controller_session='replacement-controller')
            a.snapshot = lambda: (dict(replacement, evaluated_ns=g.clock()), g.clock())
            reply = m.emergency_stop()
            assert not reply['ok']
            return False, 'controller_restarted'
        return original(p, **kw)
    m._ack = restart
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert result['controller_session'] == 'controller-1'
    assert not result['controller_gait_zero']


@pytest.mark.parametrize('which', ['controller_session', 'bridge_epoch'])
def test_superseded_success_cannot_cross_local_owner_change(rig, which):
    g, n, a, m = rig
    original = m._ack
    def changed(p, **kw):
        if p['operation'] == STOP:
            g.clock.sleep(.001)
            assert m.emergency_stop()['ok']
            if which == 'controller_session':
                m.session = 'replacement-controller'
            else:
                m.epoch = 'replacement-bridge'
            return False, 'controller_ack_timeout'
        return original(p, **kw)
    m._ack = changed
    result = m.execute(.1, 0., 0., .1)
    assert result['state'] == 'uncertain' and m.latched
    assert not result['controller_gait_zero']
