"""Real C++/generated messages and local Flask, simulated non-actuating ROS."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import urllib.error

import pytest

from champ_heartbeat import START, STOP, EMERGENCY
from test_champ_heartbeat import rig, gate, clear_lidar
from test_continuous_lidar_safety import no_network, obstruction


@pytest.fixture(autouse=True)
def isolate_imported_default_state(tmp_path, monkeypatch):
    # Some existing Cognitive imports acquire a default World Model lock.
    # Keep that offline test artifact outside the source checkout.
    monkeypatch.chdir(tmp_path)


def cognitive_imports(monkeypatch):
    monkeypatch.syspath_prepend('/tmp/mayday-cognitive-stop-wait-resume')
    for name in list(sys.modules):
        if name == 'robot_bridge' or name.startswith('robot_bridge.'):
            monkeypatch.delitem(sys.modules,name)


def load(monkeypatch):
    path = Path('/tmp/mayday-cognitive-stop-wait-resume/robot_bridge/http_response.py')
    spec = importlib.util.spec_from_file_location('recovery_http', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('axes', [(.1,0.,0.),(0.,.08,0.),(0.,-.08,0.),(0.,0.,.25),(0.,0.,-.25)])
@pytest.mark.parametrize('fault', ['intrusion', 'unavailable', 'stale'])
def test_controller_issued_stop_and_acceptance_are_exact(rig, monkeypatch, axes, fault):
    g, n, a, m = rig
    start = g.clock()
    def scan():
        sample = clear_lidar(g.clock)
        if g.clock()-start > 90_000_000:
            if fault == 'intrusion': sample = obstruction(sample)
            elif fault == 'unavailable': sample = dict(available=False)
            else: sample['age_seconds'] = .301
        return sample
    m.lidar_reader = scan
    result = m.execute(*axes, .5)
    assert result['state'] == 'interrupted' and not result['completed_progress_credit']
    result.update(automatic_stop=True, linear_x=axes[0], linear_y=axes[1], angular_z=axes[2], duration=.5)
    parser = load(monkeypatch)
    normalized = parser.server_error(503, result, endpoint='/motion')
    cert = parser.certified_active_lidar_interruption(normalized)
    assert cert, normalized
    stop = cert['stop_certificate']
    assert stop['last_operation'] == STOP and stop['generation'] == result['generation']
    assert not stop['active'] and stop['gait_zero']
    assert stop['sequence'] == n.messages[-1]['sequence']
    assert m.recovery_status()['command_pending'] is False
    assert not result['physical_stop_verified']


def test_failed_stop_never_emits_a_recovery_certificate(rig):
    g, n, a, m = rig
    start = g.clock()
    m.lidar_reader = lambda: obstruction(clear_lidar(g.clock)) if g.clock()-start > 90_000_000 else clear_lidar(g.clock)
    n.fail = lambda p:p['operation'] == STOP
    result = m.execute(.1, 0., 0., .5)
    assert result['state'] == 'uncertain' and result['stop_certificate'] is None
    assert m.recovery_status()['admission_quarantined']
    assert not m.execute(.1,0.,0.,.2)['transport_attempted']


def test_real_flask_and_cognitive_client_preserve_certificate(rig, monkeypatch):
    import app as bridge
    cognitive_imports(monkeypatch)
    from types import SimpleNamespace
    g, n, a, m = rig
    start = g.clock()
    m.lidar_reader = lambda: obstruction(clear_lidar(g.clock)) if g.clock()-start > 90_000_000 else clear_lidar(g.clock)
    monkeypatch.setattr(bridge, 'ros_ready', True)
    monkeypatch.setattr(bridge, 'publisher_node', SimpleNamespace(champ_commands=m))
    local = bridge.app.test_client()
    parser = load(monkeypatch)
    monkeypatch.setitem(sys.modules, 'robot_bridge.http_response', parser)
    path = Path('/tmp/mayday-cognitive-stop-wait-resume/robot_bridge/client.py')
    spec = importlib.util.spec_from_file_location('recovery_client', path)
    client = importlib.util.module_from_spec(spec); spec.loader.exec_module(client)
    def urlopen(req, **kw):
        reply = local.post('/motion', json=json.loads(req.data)) if req.method=='POST' else local.get('/status')
        if reply.status_code != 200:
            raise urllib.error.HTTPError(req.full_url, reply.status_code, 'offline Flask', {}, io.BytesIO(reply.data))
        stream = io.BytesIO(reply.data); stream.status = reply.status_code
        return stream
    monkeypatch.setattr(client.urllib.request, 'urlopen', urlopen)
    from robot_bridge.forward_interlock import ForwardMotionInterlock
    interlock = ForwardMotionInterlock(lambda **kw:dict(available=True,valid=True,
        producer_session='health',reason='fresh',effective_age_seconds=0.,
        sectors={'front':dict(available=True,state='CLEAR')}), expected_session='health',stop_callback=m.emergency_stop)
    interlock.refresh()
    robot = client.RobotBridgeClient(base_url='http://offline.invalid', forward_interlock=interlock)
    result = robot.move_forward(speed=.1,seconds=.5,dispatch_guard=lambda:True)
    assert parser.certified_active_lidar_interruption(result)
    assert not interlock.status()['active_forward'] and not interlock.status()['pending_forward']
    status = robot.status()['guarded_controller']
    assert status['available'] and status['gait_zero'] and not status['command_pending']
    assert n.messages[-1]['operation'] == STOP


@pytest.mark.parametrize("error", [0,120,-120])
def test_flask_client_behavior_mission_watchdog_stop_wait_reacquire_and_arrive(rig, tmp_path, monkeypatch, error):
    """Real application boundaries; camera/LiDAR delivery and ROS are simulated."""
    import app as bridge
    cognitive_imports(monkeypatch)
    from types import SimpleNamespace
    from test_find_marvin_closed_loop import make_runtime, run
    from robot_bridge.client import RobotBridgeClient
    g, n, a, m = rig
    bundle = make_runtime(tmp_path, monkeypatch, [(error,.60),(0,.60),(0,.50)])
    runtime, behavior, _, events, clock = bundle
    behavior.reacquire_find_marvin_v2 = lambda **kw:behavior.observe_find_marvin_v2()
    start = g.clock()
    def raw():
        sample = clear_lidar(g.clock)
        if (len([p for p in n.messages if p['operation']==START]) == 1
                and g.last['active'] and g.clock()-start > 90_000_000):
            sample = obstruction(sample)
        return sample
    m.lidar_reader = raw
    monkeypatch.setattr(bridge, 'ros_ready', True)
    monkeypatch.setattr(bridge, 'publisher_node', SimpleNamespace(champ_commands=m))
    local = bridge.app.test_client()
    def urlopen(req, **kw):
        # Distinct HTTP acquisitions have advancing host time, including STOPs
        # issued back-to-back while stationary. No command bound is extended.
        g.clock.sleep(.000000001)
        path = req.full_url.removeprefix('http://offline.invalid')
        assert path in ('/motion','/stop','/status')
        reply = local.post(path,json=json.loads(req.data) if req.data else {}) if req.method=='POST' else local.get(path)
        if reply.status_code != 200:
            raise urllib.error.HTTPError(req.full_url,reply.status_code,'offline Flask',{},io.BytesIO(reply.data))
        stream=io.BytesIO(reply.data);stream.status=reply.status_code
        return stream
    monkeypatch.setattr('urllib.request.urlopen',urlopen)
    robot=RobotBridgeClient(base_url='http://offline.invalid',forward_interlock=runtime.forward_interlock)
    runtime.robot_client=behavior.robot=robot
    if error:
        from behavior_manager import BehaviorManager
        behavior.execute_guarded_turn=BehaviorManager.execute_guarded_turn.__get__(behavior)
        behavior._execute_target_directed_turn=BehaviorManager._execute_target_directed_turn.__get__(behavior)
    runtime.forward_interlock.stop_callback=robot.stop
    read=behavior.lidar
    def lidar(**kw):
        if not g.last['active']:g.clock.sleep(.02)
        state=read(**kw);native=raw()
        state['bridge_scan_identity']={k:native[k] for k in ('producer_session','acquisition_sequence','source_stamp_ns')}
        state['sectors']={'front':dict(available=True,state='CLEAR')}
        return state
    runtime.world_model.get_lidar_obstacles=lidar
    runtime.forward_interlock.reader=lidar
    runtime.forward_interlock.refresh()
    # Deterministic scheduling of the existing independent health monitor.
    # No guard is bypassed; the real interlock refreshes the real mock scan.
    motion=robot.motion
    def monitored_motion(**kw):
        runtime.forward_interlock.refresh()
        return motion(**kw)
    robot.motion=monitored_motion
    outcome=run(runtime)
    assert outcome['state']=='ARRIVED', (outcome['reason'], [(h['result'].get('reason'),h['result'].get('error'),h['result'].get('approach_result',{}).get('reason')) for h in outcome['history']],outcome['stop_result'])
    assert outcome['history'][0]['result']['interrupted']
    assert outcome['completed_forward_actions']==1
    assert len([p for p in n.messages if p['operation']==START])==2
    first, second=outcome['history']
    assert not first['result']['full_step_completed']
    assert first['source_frame_stamp_ns'] < second['observation']['identity_source_frame_stamp_ns'] < second['source_frame_stamp_ns']
    assert runtime.mission_manager.get_active_mission() is None
    assert not g.last['active'] and g.last['gait_zero']
    assert outcome['lidar_recovery_history'][0]['wait_result']['ok']
