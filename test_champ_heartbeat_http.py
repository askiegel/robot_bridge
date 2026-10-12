"""Actual app handler bodies with fake Flask binding, real watchdog/messages.

No app import (it starts ROS/services); no HTTP/ROS participant is created.
Flask is unavailable in the offline Python, so this is handler integration,
not a production Flask/DDS end-to-end claim.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Twist
from champ_heartbeat import START, HEARTBEAT, STOP, EMERGENCY
from test_champ_heartbeat import rig, gate
from robot_bridge.ros_motion import RosMotion


def handlers(manager):
    parsed=ast.parse(Path('app.py').read_text())
    names={'validate_motion_payload','motion','stop_robot','streaming_motion_step',
           'local_motion_forward','navigation_start','navigation_goal',
           'mapping_navigation_start','mapping_navigation_goal'}
    body=[]
    for f in parsed.body:
        if isinstance(f,ast.FunctionDef) and f.name in names:
            f.decorator_list=[];body.append(f)
        elif isinstance(f,ast.Assign) and any(isinstance(t,ast.Name) and t.id in
                {'MAX_LINEAR_X','MAX_LINEAR_Y','MAX_ANGULAR_Z','MAX_DURATION',
                 'STREAM_DEFAULT_TIMEOUT_SECONDS','STREAM_MIN_TIMEOUT_SECONDS','STREAM_MAX_TIMEOUT_SECONDS'} for t in f.targets):
            body.append(f)
    request=SimpleNamespace(body={},get_json=lambda **kw:request.body)
    ns=dict(request=request,math=math,ros_ready=True,publisher_node=SimpleNamespace(champ_commands=manager),
            jsonify=lambda *a,**kw:a[0] if a else kw,now_iso=lambda:'synthetic-UTC')
    exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),'app.py','exec'),ns)
    return ns,request

@pytest.mark.parametrize('payload',[
    {'linear_x':.1},{'linear_x':-.1},{'linear_y':.08},{'linear_y':-.08},
    {'angular_z':.3},{'angular_z':-.3}])
def test_actual_motion_handler_delegates_bounded_commands_to_real_adapter(rig,payload):
    g,n,a,m=rig;ns,req=handlers(m);req.body=dict(payload,duration=.1)
    result,status=ns['motion']()
    assert status==200 and result['mode']=='bounded' and result['automatic_stop']
    assert result['returned_immediately'] is False and not result['physical_stop_verified']
    assert result['state']=='completed' and n.messages[-1]['operation']==STOP

@pytest.mark.parametrize('payload',[
    {'linear_x':.1,'streaming':True},{'linear_x':.1,'angular_z':.1},
    {'linear_y':.08,'angular_z':.1},{'linear_x':.51},{'linear_y':.101},
    {'angular_z':1.01},{'duration':2.01},{'duration':0},{'linear_x':'NaN'}])
def test_handler_unsupported_admission_never_publishes(rig,payload):
    g,n,a,m=rig;ns,req=handlers(m);req.body=payload
    r,status=ns['motion']();assert status in (400,409) and not r['ok'] and not n.messages

@pytest.mark.parametrize('endpoint',['navigation_start','navigation_goal',
    'mapping_navigation_start','mapping_navigation_goal','local_motion_forward'])
def test_competing_navigation_paths_are_refused_before_any_control_call(rig,endpoint):
    g,n,a,m=rig;ns,req=handlers(m)
    r,status=ns[endpoint]();assert status==409 and not r['ok'] and not n.messages


def test_handler_is_fail_closed_without_guard_capability(rig):
    g,n,a,m=rig;ns,req=handlers(m);ns['publisher_node']=SimpleNamespace();req.body={'linear_x':.1}
    assert ns['motion']()[1]==503 and not n.messages


def test_emergency_handler_revokes_before_any_further_publication(rig):
    g,n,a,m=rig;ns,req=handlers(m);req.body={'linear_x':.1,'duration':1.}
    def hook(p):
        if p['operation']==HEARTBEAT:
            n.hook=None;assert ns['stop_robot']()['ok']
    n.hook=hook;r,status=ns['motion']()
    assert status==503 and r['state']=='interrupted' and not r['completed_progress_credit']
    ix=next(i for i,p in enumerate(n.messages) if p['operation']==EMERGENCY)
    assert all(p['operation'] not in (START,HEARTBEAT) for p in n.messages[ix+1:])


def test_alternate_ros_motion_adapter_refuses_legacy_nonzero_without_initialization():
    r=RosMotion();r.initialize=lambda:(_ for _ in ()).throw(AssertionError('must not initialize ROS'))
    with pytest.raises(RuntimeError):r.publish_motion(.1,0,.2)
    messages=[];r._publisher=SimpleNamespace(publish=messages.append)
    with pytest.raises(RuntimeError):r._publish_twist(.1,0)
    r._publish_twist(0,0);assert len(messages)==1 and messages[0]==Twist()
