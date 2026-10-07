"""Offline HTTP -> actual Twist construction with a fake final ROS publisher."""
import pytest
import app as bridge
from test_motion_audit import setup_fake_egress


@pytest.fixture
def egress(monkeypatch):
    publisher = setup_fake_egress(monkeypatch)
    bridge.clear_streaming_state()
    monkeypatch.setattr(bridge.time, 'sleep', lambda _: None)
    monkeypatch.setattr(bridge, 'cancel_navigation_goal', lambda: {'active': False})
    return bridge.app.test_client(), publisher


def axes(message):
    return message.linear.x, message.linear.y, message.angular.z


@pytest.mark.parametrize('y', [.08, -.08])
def test_bounded_strafe_reaches_twist_and_auto_stops(egress, y):
    client, publisher = egress
    r = client.post('/motion', json={'linear_y': y, 'duration': .5})
    assert r.status_code == 200
    assert r.json['mode'] == 'bounded' and r.json['automatic_stop']
    assert r.json['returned_immediately'] is False
    assert r.json['linear_x'] == r.json['angular_z'] == 0
    assert r.json['linear_y'] == y
    assert [axes(m) for m in publisher.messages] == [(0, y, 0), (0, 0, 0)]
    segment = bridge.motion_audit_snapshot()['recent_segments'][-1]
    assert segment['linear_y'] == y and segment['state'] == 'completed'
    assert client.get('/status').json['motion']['linear_y'] == 0


@pytest.mark.parametrize('field,value', [('linear_x', .1), ('angular_z', -.25)])
def test_legacy_axes_and_omitted_lateral_unchanged(egress, field, value):
    client, publisher = egress
    r = client.post('/motion', json={field: value, 'duration': .5})
    assert r.status_code == 200 and r.json[field] == value
    assert r.json['linear_y'] == 0
    expected = (.1, 0, 0) if field == 'linear_x' else (0, 0, -.25)
    assert [axes(m) for m in publisher.messages] == [expected, (0, 0, 0)]


@pytest.mark.parametrize('y', [.08, -.08])
def test_streaming_lateral_status_and_stop_zero_all_axes(egress, y):
    client, publisher = egress
    r = client.post('/motion', json={'linear_y': y, 'streaming': True, 'watchdog_timeout': .5})
    assert r.status_code == 200 and r.json['returned_immediately']
    motion = client.get('/status').json['motion']
    assert motion['streaming'] and motion['linear_y'] == y
    bridge.streaming_motion_step()
    assert axes(publisher.messages[-1]) == (0, y, 0)
    assert client.post('/stop').status_code == 200
    assert axes(publisher.messages[-1]) == (0, 0, 0)
    motion = client.get('/status').json['motion']
    assert motion['linear_x'] == motion['linear_y'] == motion['angular_z'] == 0
    assert not motion['streaming']


def test_deadman_zeros_lateral_egress_and_state(egress):
    client, publisher = egress
    client.post('/motion', json={'linear_y': -.08, 'streaming': True})
    with bridge.motion_lock:
        bridge.motion_state['deadline_monotonic'] = bridge.time.monotonic() - 1
    result = bridge.streaming_motion_step()
    assert result['ok']
    assert axes(publisher.messages[-1]) == (0, 0, 0)
    m = client.get('/status').json['motion']
    assert not m['streaming'] and m['linear_x'] == m['linear_y'] == m['angular_z'] == 0
    assert m['watchdog_stop_count'] >= 1


@pytest.mark.parametrize('y', [None, True, 'bad', [], {}, 'NaN', 'Infinity', '-Infinity', .100001, -.100001])
def test_invalid_lateral_never_reaches_publisher(egress, y):
    client, publisher = egress
    r = client.post('/motion', json={'linear_y': y})
    assert r.status_code == 400 and not r.json['ok']
    assert publisher.messages == []


@pytest.mark.parametrize('axis', ['linear_x', 'angular_z'])
def test_lateral_cannot_silently_mix_axes(egress, axis):
    client, publisher = egress
    assert client.post('/motion', json={'linear_y': .08, axis: .1}).status_code == 400
    assert not publisher.messages


@pytest.mark.parametrize('duration', [0, -.1, 2.0001])
def test_existing_duration_bounds_preserved(egress, duration):
    client, publisher = egress
    assert client.post('/motion', json={'linear_y': .08, 'duration': duration}).status_code == 400
    assert not publisher.messages


def test_status_exposes_bounded_strafe_while_request_waits(egress, monkeypatch):
    client, publisher = egress
    seen = []
    monkeypatch.setattr(bridge.time, 'sleep', lambda _: seen.append(client.get('/status').json['motion']))
    assert client.post('/motion', json={'linear_y': .08, 'duration': .5}).status_code == 200
    assert seen[0]['linear_y'] == .08 and seen[0]['linear_x'] == seen[0]['angular_z'] == 0
    assert seen[-1]['linear_y'] == 0


def test_lateral_capability_advertised_with_bridge_limit(egress):
    client,_=egress
    assert client.get('/status').json['motion_capabilities']=={'linear_y':True,'max_linear_y':.1}


@pytest.mark.parametrize('y',[.10,-.10])
def test_bridge_lateral_ceiling_is_supported_without_using_controller_max(egress,y):
    client,publisher=egress
    assert client.post('/motion',json={'linear_y':y,'duration':.5}).status_code==200
    assert axes(publisher.messages[0])==(0,y,0)


def test_explicit_stop_during_bounded_strafe_never_resumes(egress,monkeypatch):
    client,publisher=egress
    def stop_while_waiting(seconds):
        if seconds == .5:
            assert client.post('/stop').json['ok']
            assert axes(publisher.messages[-1])==(0,0,0)
    monkeypatch.setattr(bridge.time,'sleep',stop_while_waiting)
    result=client.post('/motion',json={'linear_y':.08,'duration':.5})
    assert result.status_code==200
    assert axes(publisher.messages[0])==(0,.08,0)
    assert all(axes(m)==(0,0,0) for m in publisher.messages[1:])
    assert len(publisher.messages)>=3
