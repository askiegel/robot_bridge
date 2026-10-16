"""Offline native-end classification: real pinned C++ gate, no ROS participant."""
import ast
import copy
import json
import os
from pathlib import Path
import sys

import pytest
import champ_heartbeat as bridge
from champ_heartbeat import START, HEARTBEAT, STOP, EMERGENCY
from test_champ_heartbeat import gate, rig
from test_continuous_lidar_safety import no_network

AXES = [(0.1,0.,0.),(-0.1,0.,0.),(0.,.08,0.),(0.,-.08,0.),(0.,0.,.25),(0.,0.,-.25)]


def baseline_manager(rig):
    path=os.environ.get('MAYDAY_DEADLINE_CLASSIFICATION_BASELINE')
    if not path: pytest.skip('retained source required for counterfactual')
    tree=ast.parse(Path(path).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='HeartbeatCommands')
    namespace=dict(vars(bridge))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),path,'exec'),namespace)
    g,_,adapter,manager=rig
    return namespace['HeartbeatCommands'](adapter,'host-1',clock=g.clock,sleep=g.clock.sleep,
        epoch='bridge-1',lidar_reader=manager.lidar_reader)


def cross_boundary(rig, manager, phase, offset, fault=None):
    g,node,adapter,_=rig
    status=manager._status; injected=[]; terminal=[]
    def observe():
        c=manager.current; frame=sys._getframe(1)
        if c and not c['retired'] and not injected:
            remaining=c['end']-g.clock()
            in_loop=frame.f_code.co_name=='execute'
            in_ack=frame.f_code.co_name=='_ack' and node.messages[-1]['operation']==HEARTBEAT
            if ((phase=='loop' and in_loop and 0<remaining<=5_000_000)
                    or (phase=='ack' and in_ack and 0<remaining<=100_000_000)):
                injected.append({'remaining_ns':remaining,'caller':frame.f_code.co_name,
                                 'end':c['end'],'before_ns':g.clock()})
                if fault=='early_stop': manager.emergency_stop()
                g.clock.now=c['end']+offset
                if fault=='lidar':
                    c['safety_failure']='active_lidar_intrusion'
                    c['lidar_safety']=dict(c['lidar_safety'],permitted=False,reason='active_lidar_intrusion')
                if fault=='missing_acceptance': c.pop('acceptance',None)
        result=status()
        if c and result and not result.get('active') and result.get('generation')==c['generation']:
            terminal.append(copy.deepcopy(result))
        return result
    manager._status=observe
    return injected,terminal


@pytest.mark.parametrize('baseline',[True,False])
@pytest.mark.parametrize('axes',AXES)
@pytest.mark.parametrize('phase',['loop','ack'])
@pytest.mark.parametrize('offset',[-1,0,1,6_689_554,20_000_000])
def test_same_native_end_has_one_completion_rule(rig,baseline,axes,phase,offset,record_property):
    g,node,adapter,manager=rig
    if baseline: manager=baseline_manager(rig)
    injected,terminal=cross_boundary(rig,manager,phase,offset)
    result=manager.execute(*axes,.5)
    assert injected
    should_complete=not baseline or offset<0
    assert result['state']==('completed' if should_complete else 'interrupted'),result
    assert result['completed_progress_credit'] is should_complete
    assert result['reason']==('bounded_deadline_completed' if should_complete else 'command_deadline_expired')
    assert result['controller_gait_zero'] and not manager.latched
    assert result['stop_reason']=='scoped_gait_zero'
    assert not g.last['active'] and g.last['gait_zero']
    assert any(x['reason']=='command_deadline_expired' for x in terminal)
    p=result['stop_certificate'];assert p['last_request_accepted'] and p['gait_zero']
    assert p['last_request_issued_ns']>=result['deadline_ns']
    assert p['generation']==result['generation'] and p['last_request_generation']==result['generation']
    assert result['controller_acceptance']['last_request_accepted']
    assert all(p['issued_ns']<p['deadline_ns'] for p in node.messages if p['operation'] in (START,HEARTBEAT))
    assert len({p['deadline_ns'] for p in node.messages if p['operation'] in (START,HEARTBEAT,STOP)})==1
    record_property('deadline_boundary',json.dumps({'baseline':baseline,'axes':axes,'phase':phase,
        'offset_ns':offset,'result':result,'injected':injected}))


@pytest.mark.parametrize('phase',['loop','ack'])
@pytest.mark.parametrize('fault',['early_stop','lidar'])
def test_crossing_end_does_not_normalize_real_interruptions(rig,phase,fault):
    g,node,_,manager=rig
    cross_boundary(rig,manager,phase,6_689_554,fault)
    result=manager.execute(.1,0.,0.,.5)
    assert result['state']!='completed' and not result['completed_progress_credit']
    assert result['controller_gait_zero'] and not g.last['active']
    if fault=='early_stop':
        assert result['reason']=='command_cancelled'
        emergency=next(i for i,p in enumerate(node.messages) if p['operation']==EMERGENCY)
        assert all(p['operation'] not in (START,HEARTBEAT) for p in node.messages[emergency+1:])
    else: assert result['reason']=='active_lidar_intrusion'


@pytest.mark.parametrize('fault',['missing_acceptance','ack_session','ack_epoch','ack_generation',
    'ack_sequence','ack_operation','ack_refused','ack_tail_expired','ack_issue_future','ack_evaluation',
    'send_disagreement','native_epoch','native_generation','native_sequence','native_deadline','native_session','native_missing',
    'receipt_expired','native_emergency','unelapsed_deadline','invalid_lidar'])
def test_completion_requires_exact_current_native_and_accepted_receipt(rig,fault):
    g,node,adapter,manager=rig
    cross_boundary(rig,manager,'loop',6_689_554)
    complete=manager._completion_outcome
    def validate(c):
        a=c['acceptance']
        if fault=='missing_acceptance': c.pop('acceptance')
        if fault=='ack_session': a['controller_session']='wrong'
        if fault=='ack_epoch': a['bridge_epoch']='wrong'
        if fault=='ack_generation': a['last_request_generation']+=1
        if fault=='ack_sequence': a['last_request_sequence']-=1
        if fault=='ack_operation': a['last_operation']=STOP
        if fault=='ack_refused': a['last_request_accepted']=False
        if fault=='ack_tail_expired': a['last_request_issued_ns']=c['end']-250_000_000
        if fault=='ack_tail_expired': c['last_send']=a['last_request_issued_ns']
        if fault=='ack_issue_future': a['last_request_issued_ns']=c['last_send']=c['end']
        if fault=='send_disagreement': c['last_send']-=1
        if fault=='ack_evaluation': a['evaluated_ns']=g.clock()+1
        if fault=='invalid_lidar': c['lidar_safety']['permitted']=False
        if fault=='unelapsed_deadline':
            # A reason string alone cannot substitute for the owned end clock.
            g.clock.now=c['end']-1
        status=manager._status
        def changed():
            s=status()
            if fault=='native_missing':return None
            if fault=='native_session':s['controller_session']='changed'
            if fault=='native_epoch':s['bridge_epoch']='changed'
            if fault=='native_generation':s['generation']+=1
            if fault=='native_sequence':s['sequence']-=1
            if fault=='native_deadline':s['deadline_ns']+=1
            if fault in {'receipt_expired','native_emergency'}:
                s['active']=False
                s['reason']='command_receipt_expired' if fault=='receipt_expired' else 'emergency_gait_zero'
            return s
        manager._status=changed
        try:return complete(c)
        finally:manager._status=status
    manager._completion_outcome=validate
    result=manager.execute(.1,0.,0.,.5)
    assert result['state']!='completed' and not result['completed_progress_credit']
    assert not g.last['active'] and g.last['gait_zero']
    if fault in {'native_session','native_missing'}:assert manager.latched


@pytest.mark.parametrize('failure',['missing','late','uncertain','duplicate','superseded'])
def test_normal_expiry_still_requires_matching_timely_scoped_stop(rig,failure):
    g,node,adapter,manager=rig
    cross_boundary(rig,manager,'ack',6_689_554)
    publish=node.publish
    def stop_packet(msg):
        if msg.operation!=STOP:return publish(msg)
        if failure=='missing':return
        if failure=='late':
            g.clock.sleep(.251)
            return publish(msg)
        publish(msg)
        if failure=='uncertain':raise RuntimeError('STOP publication uncertain')
        if failure=='duplicate':publish(msg)
        if failure=='superseded':assert manager.emergency_stop()['ok']
    node.publish=stop_packet
    result=manager.execute(.1,0.,0.,.5)
    assert not result['completed_progress_credit'] and result['state']!='completed'
    assert not g.last['active'] and g.last['gait_zero']
    if failure!='superseded':
        assert manager.latched
        assert manager.execute(.1,0.,0.,.5)['state']=='refused'
    else:assert result['reason']=='command_cancelled'


def test_old_deadline_finish_cannot_write_or_stop_successor(rig):
    g,node,_,manager=rig
    prior=[]
    node.hook=lambda p:prior.append(manager.current) if p['operation']==START else None
    assert manager.execute(.1,0.,0.,.5)['completed_progress_credit']
    def successor(p):
        if p['operation']!=START:return
        current=manager.current;packets=len(node.messages)
        result=manager._finish(prior[0],'interrupted','command_deadline_expired',True)
        assert result['state']=='refused' and not result['completed_progress_credit']
        assert manager.current is current and len(node.messages)==packets
        assert g.last['active'] and not manager.latched
    node.hook=successor
    result=manager.execute(0.,.08,0.,.5)
    assert result['state']=='completed' and result['completed_progress_credit']


def test_real_cognitive_decoder_observes_completion_and_cancellation_accounting(rig,monkeypatch):
    # Read-only existing Cognitive integration harness; production source unchanged.
    from test_bridge_active_lidar_ownership import connected,move
    client,interlock,_,requests=connected(rig,monkeypatch)
    g,node,_,manager=rig
    cross_boundary(rig,manager,'ack',6_689_554)
    result=move(client,(.1,0.,0.),.5)
    assert result['ok'] and result['evidence_valid'] and result['completed_progress_credit']
    assert result['physical_stop_verified'] is False
    first=requests.count(('POST','/motion'))
    def stop(p):
        if p['operation']==HEARTBEAT:
            node.hook=None
            assert manager.emergency_stop()['ok']
    node.hook=stop
    assert interlock.refresh()[0]
    interrupted=move(client,(.1,0.,0.),.5)
    assert not interrupted['ok'] and interrupted.get('completed_progress_credit') is False, interrupted
    assert interrupted['state']=='interrupted'
    assert requests.count(('POST','/motion'))==first+1


@pytest.mark.parametrize('delay_ns',[249_999_999,250_000_000,250_000_001])
def test_stop_status_read_cannot_cross_original_ack_limit_and_still_complete(rig,delay_ns):
    g,node,_,manager=rig
    cross_boundary(rig,manager,'ack',6_689_554)
    status=manager._status; delayed=[]
    def observed():
        record=manager.stop_request
        if (record and record['packet']['operation']==STOP and record['result'] is None
                and sys._getframe(1).f_code.co_name=='_ack' and not delayed):
            delayed.append(True)
            g.clock.now=record['packet']['issued_ns']+delay_ns
        return status()
    manager._status=observed
    result=manager.execute(.1,0.,0.,.5)
    assert delayed and not g.last['active'] and g.last['gait_zero']
    timely=delay_ns<250_000_000
    assert result['completed_progress_credit'] is timely
    assert (result['state']=='completed') is timely
    assert manager.latched is (not timely)
    if not timely:assert result['stop_reason']=='controller_ack_timeout'
