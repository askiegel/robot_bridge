"""Offline: real watchdog/generated messages, simulated non-actuating ROS I/O."""
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace
import urllib.error

import pytest
from champ_heartbeat import START, HEARTBEAT, STOP, EMERGENCY
from lidar_telemetry import LidarTelemetry
from local_motion_guard import bounded_command_clearance, protected_sweep
from test_champ_heartbeat import rig, gate, clear_lidar

AXES = [(.1, 0., 0.), (-.1, 0., 0.), (0., .08, 0.), (0., -.08, 0.),
        (0., 0., .25), (0., 0., -.25)]


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def refuse(*a, **kw):
        raise AssertionError('Offline test attempted network I/O')
    monkeypatch.setattr(socket, 'create_connection', refuse)
    monkeypatch.setattr(socket.socket, 'connect', refuse)
    # Generated message serialization is exercised without rclpy.init/DDS.
    import rclpy
    monkeypatch.setattr(rclpy, 'init', refuse)


def obstruction(sample):
    sample = copy.deepcopy(sample)
    # Robot front at ~0.32m after the calibrated lidar-to-base offset.
    sample['scan']['ranges'][20] = .40
    return sample


@pytest.mark.parametrize('axes', AXES)
@pytest.mark.parametrize('fault', ['intrusion', 'stale', 'frozen'])
def test_active_intrusion_or_sensor_loss_interrupts_all_directions(rig, axes, fault):
    g, n, a, m = rig
    start = g.clock()
    frozen = clear_lidar(g.clock)
    def read():
        sample = clear_lidar(g.clock)
        if fault == 'frozen':
            sample = copy.deepcopy(frozen)
            sample['age_seconds'] = g.clock()/1e9 - sample['received_monotonic_seconds']
        elif g.clock() - start > 90_000_000:
            if fault == 'intrusion':
                sample = obstruction(sample)
            else:
                sample['age_seconds'] = .301
        return sample
    m.lidar_reader = read
    r = m.execute(*axes, .8)
    assert r['state'] == 'interrupted'
    assert r['reason'] == ('active_lidar_intrusion' if fault == 'intrusion' else 'active_lidar_stale')
    assert r['transport_attempted'] and r['controller_gait_zero']
    assert not r['completed_progress_credit'] and not r['physical_stop_verified']
    assert n.messages[-1]['operation'] == STOP and not g.last['active']
    assert (g.last['linear_x'], g.last['linear_y'], g.last['angular_z']) == (0, 0, 0)
    assert r['lidar_safety']['evaluated_ns'] < r['deadline_ns']
    assert len({p['deadline_ns'] for p in n.messages if p['operation'] in (START, HEARTBEAT)}) == 1


@pytest.mark.parametrize('axes', AXES)
def test_initial_intrusion_is_zero_transport(rig, axes):
    g, n, a, m = rig
    m.lidar_reader = lambda: obstruction(clear_lidar(g.clock))
    r = m.execute(*axes, .2)
    assert r['state'] == 'refused' and r['transport_attempted'] is False
    assert not r['completed_progress_credit'] and not n.messages


@pytest.mark.parametrize('point', [(0, .44), (0, -.44), (-.44, 0), (.44, 0)])
@pytest.mark.parametrize('axes', AXES)
def test_full_starting_circle_has_no_directional_exemption(point, axes):
    r = protected_sweep([point], [10]*8, axes, .5)
    assert not r['permitted'] and r['reason'] == 'active_lidar_intrusion'
    assert r['protected_radius_m'] == .45


@pytest.mark.parametrize('axes', AXES)
@pytest.mark.parametrize('distance', [.45, .450001])
def test_exact_protected_radius_boundary_is_not_relaxed(axes, distance):
    point = (0, distance) if axes[0] else (distance, 0)
    r = protected_sweep([point], [10]*8, axes, .2)
    assert r['permitted'] is (distance > .45)


@pytest.mark.parametrize('axes', AXES)
def test_clear_circle_and_current_frame_remaining_sweep(axes):
    point = (0, .46) if axes[0] else (.46, 0)
    assert protected_sweep([point], [10]*8, axes, .5)['permitted']
    if axes[0] or axes[1]:
        v = axes[0] or axes[1]
        endpoint_point = (.45 + abs(v)*.25, 0) if axes[0] else (0, .45 + abs(v)*.25)
        if v < 0:
            endpoint_point = tuple(-p for p in endpoint_point)
        assert not protected_sweep([endpoint_point], [10]*8, axes, .5)['permitted']
        assert protected_sweep([endpoint_point], [10]*8, axes, .1)['permitted']


def test_scan_transform_matches_existing_cognitive_geometry():
    # Import the existing pure builder only; no runtime/provider/ROS startup.
    cognitive = Path('/home/tkieg/robot_services/cognitive_deployed')
    sys.path.insert(0, str(cognitive))
    try:
        from local_motion_safety_envelope import build_local_motion_lidar_geometry
    finally:
        sys.path.pop(0)
    sample = clear_lidar(lambda: 1_000_000_000)
    geom = build_local_motion_lidar_geometry(sample['scan'])
    pts = [(p['x_m'], p['y_m']) for p in geom['points']]
    counts = [s['valid_sample_count'] for s in geom['sectors'].values()]
    for axes in AXES:
        expected = protected_sweep(pts, counts, axes, .5)
        actual = bounded_command_clearance(sample, axes, .5)
        assert actual == expected


@pytest.mark.parametrize('fault,reason', [
    ('coverage', 'active_lidar_coverage_missing'), ('frame', 'active_lidar_frame_invalid'),
    ('unavailable', 'active_lidar_unavailable'), ('session', 'active_lidar_session_changed'),
    ('regressed', 'active_lidar_source_regressed'), ('receipt', 'active_lidar_frozen_rejuvenation'),
    ('exception', 'active_lidar_unavailable'), ('malformed', 'active_lidar_geometry_invalid'),
    ('overflow', 'active_lidar_geometry_invalid'),
])
def test_active_invalid_sensing_has_no_completed_credit(rig, fault, reason):
    g, n, a, m = rig
    saved = clear_lidar(g.clock)
    start = g.clock()
    def read():
        s = clear_lidar(g.clock)
        if g.clock() - start > 90_000_000:
            if fault == 'coverage': s['scan']['ranges'][:20] = [None]*20
            elif fault == 'frame': s['scan']['frame_id'] = 'wrong'
            elif fault == 'unavailable': s['available'] = False
            elif fault == 'session': s['producer_session'] = 'changed'
            elif fault == 'regressed': s['source_stamp_ns'] = 0
            elif fault == 'receipt':
                s = copy.deepcopy(saved)
                # Prior successful reading also must have this source identity.
                s['received_monotonic_seconds'] = g.clock()/1e9 - .01
            elif fault == 'exception': raise RuntimeError('sensor read failed')
            elif fault == 'malformed': s['scan']['angle_increment'] = 0
            elif fault == 'overflow':
                s['scan']['angle_min'] = s['scan']['angle_increment'] = 1e308
        elif fault == 'receipt':
            s = copy.deepcopy(saved)
            s['age_seconds'] = g.clock()/1e9 - s['received_monotonic_seconds']
        return s
    m.lidar_reader = read
    r = m.execute(.1, 0, 0, .8)
    assert r['reason'] == reason and r['state'] == 'interrupted'
    assert r['controller_gait_zero'] and not r['completed_progress_credit']


def test_slow_reader_cannot_release_stale_admission(rig):
    g, n, a, m = rig
    def read():
        s = clear_lidar(g.clock)
        s['age_seconds'] = .29
        g.clock.sleep(.02)
        return s
    m.lidar_reader = read
    assert m.execute(.1, 0, 0, .2)['reason'] == 'active_lidar_stale'
    assert not n.messages


def test_missing_reader_is_fail_closed(rig):
    g, n, a, m = rig
    m.lidar_reader = None
    assert m.execute(.1, 0, 0, .2)['state'] == 'refused'
    assert not n.messages


def test_failed_scoped_stop_latches_admission(rig):
    g, n, a, m = rig
    start = g.clock()
    m.lidar_reader = lambda: (obstruction(clear_lidar(g.clock)) if g.clock()-start > 90_000_000 else clear_lidar(g.clock))
    n.fail = lambda p: p['operation'] == STOP
    r = m.execute(.1, 0, 0, .8)
    assert r['state'] == 'uncertain' and not r['controller_gait_zero']
    assert m.latched and not r['completed_progress_credit']
    assert m.execute(0, .08, 0, .2)['reason'] == 'owner_busy_or_unresolved'


def test_old_safety_read_stop_and_completion_cannot_revoke_successor(rig):
    g, n, a, m = rig
    old = []
    n.hook = lambda p: old.append(m.current) if p['operation'] == START else None
    assert m.execute(.1, 0, 0, .1)['state'] == 'completed'
    def successor(p):
        if p['operation'] == START:
            current, count = m.current, len(n.messages)
            assert not m._check_safety(old[0])
            m._stop(old[0])
            m._finish(old[0], 'interrupted', 'late_sensor_failure', True)
            assert m.current is current and len(n.messages) == count and not m.latched
    n.hook = successor
    assert m.execute(0, -.08, 0, .1)['state'] == 'completed'


def test_reader_cancellation_cannot_restore_command(rig):
    g, n, a, m = rig
    cancelled = []
    def read():
        if m.current and g.clock() - m.current['start'] >= 90_000_000 and not cancelled:
            cancelled.append(m.emergency_stop())
        return clear_lidar(g.clock)
    m.lidar_reader = read
    r = m.execute(.1, 0, 0, .8)
    assert cancelled[0]['ok'] and r['state'] == 'interrupted'
    assert r['reason'] == 'command_cancelled' and not r['completed_progress_credit']
    ix = next(i for i,p in enumerate(n.messages) if p['operation'] == EMERGENCY)
    assert not any(p['operation'] in (START, HEARTBEAT) for p in n.messages[ix+1:])


def test_competing_dispatch_during_read_is_refused(rig):
    g, n, a, m = rig
    results = []
    def read():
        if m.current and not results:
            results.append(m.execute(0, .08, 0, .1))
        return clear_lidar(g.clock)
    m.lidar_reader = read
    assert m.execute(.1, 0, 0, .1)['state'] == 'completed'
    assert results[0]['reason'] == 'owner_busy_or_unresolved'


def test_contradictory_gait_zero_cannot_verify_stop(rig):
    g, n, a, m = rig
    original = a.snapshot
    def wrong_zero():
        s, received = original()
        if s.get('last_operation') == STOP:
            s = dict(s, linear_y=.1)
        return s, received
    a.snapshot = wrong_zero
    r = m.execute(.1, 0, 0, .1)
    assert r['state'] == 'uncertain' and not r['controller_gait_zero']
    assert m.latched and not r['completed_progress_credit']


def test_lost_stop_acknowledgement_cannot_certify_completion_or_admit_successor(rig):
    g, n, a, m = rig
    original = a.snapshot
    def lost_ack():
        s, received = original()
        if s.get('last_operation') == STOP:
            s = dict(s, last_request_issued_ns=s['last_request_issued_ns']-1)
        return s, received
    a.snapshot = lost_ack
    result = m.execute(.1, 0, 0, .1)
    assert result['state'] == 'uncertain' and not result['controller_gait_zero']
    assert not result['physical_stop_verified'] and not result['completed_progress_credit']
    assert m.latched and m.execute(.1, 0, 0, .1)['state'] == 'refused'


@pytest.mark.parametrize('field,value', [('bridge_epoch','another-bridge'), ('generation', 999), ('sequence', 999)])
def test_scoped_stop_ack_requires_exact_current_owner(rig, field, value):
    g, n, a, m = rig
    original = a.snapshot
    def wrong_owner():
        s, received = original()
        if s.get('last_operation') == STOP:
            s = dict(s, **{field: value})
        return s, received
    a.snapshot = wrong_owner
    result = m.execute(.1, 0, 0, .1)
    assert result['state'] == 'uncertain' and not result['controller_gait_zero']
    assert result['stop_reason'] == 'controller_command_owner_changed'
    assert not result['physical_stop_verified'] and not result['completed_progress_credit']
    assert m.latched


def test_october6_ordered_bounded_command_regression(rig):
    fixture = json.loads((Path(__file__).parent/'test_fixtures/october6_marvin_53532164.json').read_text())
    g, n, a, m = rig
    assert fixture['historical_terminal'] == 'ARRIVED' and len(fixture['actions']) == 43
    prior = 0
    for action in fixture['actions']:
        assert action['source_stamp'] > prior
        prior = action['source_stamp']
        if action['state'] == 'ADVANCING':
            assert action['tracker_quality'] >= .8 and action['source_stamp_consumed']
            assert action['retained_jit_full_capsule_check']['permitted']
        result = m.execute(*action['axes'], action['duration'])
        assert result['state'] == 'completed' and result['controller_gait_zero']
        assert result['completed_progress_credit'] and not result['physical_stop_verified']
    # Advancing scan delivery is simulated. Historical active-time scans and
    # turn JIT point sets are absent; no exact live-geometry replay is claimed.
    starts = [p for p in n.messages if p['operation'] == START]
    assert len(starts) == 43 and len({p['generation'] for p in starts}) == 43
    assert n.messages[-1]['operation'] == STOP and not g.last['active']


def message(ns):
    return SimpleNamespace(header=SimpleNamespace(frame_id='lidar_link',
        stamp=SimpleNamespace(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)),
        angle_min=-math.pi, angle_max=math.pi, angle_increment=math.tau/80,
        time_increment=0., scan_time=.1, range_min=.02, range_max=8., ranges=[2.]*80)


def test_busy_lidar_cache_cannot_block_bridge_emergency_stop(rig):
    g, n, a, m = rig
    cache = LidarTelemetry(monotonic_clock=lambda: g.clock()/1e9)
    cache.update(message(100))
    held, reading, stopped = threading.Event(), threading.Event(), threading.Event()
    answers = {}
    def read():
        if held.is_set(): reading.set()
        return cache.snapshot()
    m.lidar_reader = read
    def publish(p):
        if p['operation'] == START:
            cache._lock.acquire()
            held.set()
    n.hook = publish
    def command(): answers['command'] = m.execute(.1, 0, 0, 1.)
    def stop():
        answers['stop'] = m.emergency_stop()
        stopped.set()
    runner, stopper = threading.Thread(target=command), threading.Thread(target=stop)
    runner.start()
    try:
        assert held.wait(1.) and reading.wait(1.)
        stopper.start()
        assert stopped.wait(.5), 'LiDAR cache lock blocked emergency STOP'
        assert answers['stop']['ok']
    finally:
        if cache._lock.locked(): cache._lock.release()
        runner.join(2.)
        if stopper.ident is not None: stopper.join(2.)
    assert not runner.is_alive() and not stopper.is_alive()
    assert answers['command']['state'] == 'interrupted'
    assert not answers['command']['completed_progress_credit']
    assert not g.last['active']


def test_controller_expiry_is_independent_of_lidar_cache_lock(gate):
    cache = LidarTelemetry(monotonic_clock=lambda: gate.clock()/1e9)
    cache.update(message(100))
    assert gate.sync()['accepted'] and gate.start()['accepted']
    with cache._lock:
        assert not cache.snapshot()['available']
        gate.clock.sleep(.26)
        state = gate.tick()  # Real C++ watchdog, no Bridge/cache dependency.
        assert not state['active'] and state['reason'] == 'command_receipt_expired'
        assert state['linear_x'] == state['linear_y'] == state['angular_z'] == 0


def test_oversized_scan_never_enters_the_cached_geometry():
    cache = LidarTelemetry(monotonic_clock=lambda: 10.)
    cache.update(message(100))
    old = cache.snapshot()
    large = message(101)
    large.ranges = [2.] * 8193
    cache.update(large)
    current = cache.snapshot()
    assert current['currentness_fault'] == 'active_lidar_geometry_invalid'
    assert current['acquisition_sequence'] == old['acquisition_sequence']
    assert len(current['scan']['ranges']) == 80
    assert not bounded_command_clearance(current, AXES[0], .5)['permitted']


def test_status_delay_cannot_publish_heartbeat_after_safety_check_is_due(rig):
    g, n, a, m = rig
    delayed = []
    m.lidar_reader = lambda: obstruction(clear_lidar(g.clock)) if delayed else clear_lidar(g.clock)
    original = a.snapshot
    def snapshot():
        c = m.current
        if c and c.get('acceptance') and g.clock()-c['start'] >= 50_000_000 and not delayed:
            g.clock.sleep(.06)
            delayed.append(True)
        return original()
    a.snapshot = snapshot
    result = m.execute(.1, 0, 0, .8)
    assert delayed and result['reason'] == 'active_lidar_intrusion'
    assert result['state'] == 'interrupted' and result['controller_gait_zero']
    assert not any(p['operation'] == HEARTBEAT for p in n.messages)
    assert not result['completed_progress_credit']


def test_duplicate_callbacks_do_not_rejuvenate_frozen_scan():
    now = [10.]
    t = LidarTelemetry(monotonic_clock=lambda: now[0])
    t.update(message(100))
    now[0] += .2
    t.update(message(100))
    a = t.snapshot()
    assert a['acquisition_sequence'] == 1 and a['age_seconds'] == pytest.approx(.2)
    now[0] += .11
    t.update(message(100))
    assert not bounded_command_clearance(t.snapshot(), AXES[0], .2)['permitted']
    t.update(message(101))
    assert t.snapshot()['acquisition_sequence'] == 2
    assert bounded_command_clearance(t.snapshot(), AXES[0], .2)['permitted']


def test_regressed_or_malformed_source_fails_closed():
    t = LidarTelemetry(monotonic_clock=lambda: 10.)
    t.update(message(100))
    t.update(message(99))
    assert t.snapshot()['currentness_fault'] == 'active_lidar_source_regressed'
    assert t.snapshot()['acquisition_sequence'] == 1
    bad = message(101)
    bad.header.stamp.nanosec = -1
    t.update(bad)
    assert t.snapshot()['currentness_fault'] == 'active_lidar_source_stamp_invalid'


def test_duplicate_timestamp_with_changed_geometry_is_not_a_fresh_scan():
    t = LidarTelemetry(monotonic_clock=lambda: 10.)
    t.update(message(100))
    before = t.snapshot()
    bad = message(100)
    bad.ranges[20] = .4
    t.update(bad)
    after = t.snapshot()
    assert after['currentness_fault'] == 'active_lidar_source_stamp_reused'
    assert after['producer_session'] != before['producer_session']
    assert after['acquisition_sequence'] == before['acquisition_sequence']
    assert not bounded_command_clearance(after, AXES[0], .2)['permitted']


def test_fault_then_fresh_scan_between_polls_changes_intake_epoch(rig):
    g, n, a, m = rig
    t = LidarTelemetry(monotonic_clock=lambda: g.clock()/1e9)
    t.update(message(100))
    initial = t.snapshot()
    # An observed bad source cannot disappear into a later healthy snapshot.
    t.update(message(99))
    t.update(message(101))
    m.lidar_reader = t.snapshot
    decision = m._lidar_decision(AXES[0], .2, initial)
    assert decision['reason'] == 'active_lidar_session_changed'
    assert not decision['permitted']


@pytest.mark.parametrize('interrupt', [False, True])
def test_real_flask_routes_and_cognitive_http_client_preserve_outcome(rig, monkeypatch, interrupt):
    # Actual Flask application/route bindings; main()/ROS/servo nodes never run.
    import app as bridge
    g, n, a, m = rig
    monkeypatch.setattr(bridge, 'ros_ready', True)
    monkeypatch.setattr(bridge, 'publisher_node', SimpleNamespace(champ_commands=m))
    if interrupt:
        start = g.clock()
        m.lidar_reader = lambda: obstruction(clear_lidar(g.clock)) if g.clock()-start > 90_000_000 else clear_lidar(g.clock)
    root = Path('/tmp/mayday-cognitive-minimal-continuous-lidar-safety')
    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module
    load('robot_bridge.http_response', root/'robot_bridge/http_response.py')
    client_module = load('tested_cognitive_client', root/'robot_bridge/client.py')
    local = bridge.app.test_client()
    def urlopen(req, **kw):
        assert req.full_url == 'http://offline.invalid/motion' and req.method == 'POST'
        response = local.post('/motion', json=json.loads(req.data))
        if response.status_code != 200:
            raise urllib.error.HTTPError(req.full_url, response.status_code, 'simulated Flask HTTP', {}, io.BytesIO(response.data))
        stream = io.BytesIO(response.data)
        stream.status = response.status_code
        return stream
    monkeypatch.setattr(client_module.urllib.request, 'urlopen', urlopen)
    interlock_module = load('tested_cognitive_interlock', root/'robot_bridge/forward_interlock.py')
    health = interlock_module.ForwardMotionInterlock(
        lambda **kw: dict(available=True, valid=True, reason='fresh', producer_session='health-1',
                         effective_age_seconds=.01, sectors={'front': dict(available=True, state='CLEAR')}),
        expected_session='health-1', stop_callback=m.emergency_stop)
    health.refresh()
    # Exact client.motion -> interlock -> HTTP/error-body -> real Flask route.
    # All transport is test-client/simulated ROS; no HTTP socket or DDS participant.
    result = client_module.RobotBridgeClient(base_url='http://offline.invalid',
        forward_interlock=health).motion(linear_x=.1, duration=.5, dispatch_guard=lambda: True)
    assert result['state'] == ('interrupted' if interrupt else 'completed')
    assert result['evidence_valid'] and result['controller_gait_zero']
    assert result['completed_progress_credit'] is (not interrupt)
    if interrupt:
        assert result['reason'] == 'active_lidar_intrusion' and result['http_status'] == 503
    assert not health.status()['active_forward'] and not health.status()['pending_forward']
