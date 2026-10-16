"""Exact real C++ controller acceptance; non-actuating serialized ROS adapter."""
import pytest
from champ_heartbeat import HeartbeatCommands, START, HEARTBEAT, STOP, EMERGENCY, TIMEOUT_NS
from champ_ros_adapter import ChampTransport, GuardedCommandStatus
from test_champ_heartbeat import Clock, Gate, SimNode, gate, rig


def emit(adapter, state):
    msg = GuardedCommandStatus()
    for key in msg.get_fields_and_field_types():
        if key != 'velocity':
            setattr(msg, key, state[key])
    msg.velocity.linear.x = float(state['linear_x'])
    msg.velocity.linear.y = float(state['linear_y'])
    msg.velocity.angular.z = float(state['angular_z'])
    adapter.receive(msg)


def boundary(g, n, a, m, delay=.005):
    assert g.sync()['accepted']
    assert g.start(deadline_ns=g.clock()+200_000_000)['accepted']
    g.clock.sleep(.1998)
    p = dict(g.start_packet, operation=HEARTBEAT, sequence=2, issued_ns=g.clock())
    state = g.send(p)
    assert state['last_request_accepted'] and state['active']
    g.clock.sleep(delay)
    expired = g.tick()
    assert not expired['active'] and expired['last_request_accepted']
    emit(a, expired)
    # Freeze delivery at the explicitly injected real-controller statuses.
    a.snapshot = lambda: ChampTransport.snapshot(a)
    m.session = g.session
    return p, state, expired


@pytest.mark.parametrize('delay', [.001, .005, .05, .2])
def test_acceptance_survives_exact_active_window_and_delayed_poll(rig, delay):
    g, n, a, m = rig
    p, active, expired = boundary(*rig, delay=delay)
    before = g.clock()
    assert m._ack(p) == (True, 'command_deadline_expired')
    assert g.clock() == before
    retained = a.acknowledgement(p)
    assert retained['last_request_accepted']
    assert retained['last_request_issued_ns'] == p['issued_ns']
    assert retained['last_request_sequence'] == p['sequence']
    assert retained['gait_zero'] and not retained['active']


def test_delayed_status_arrival_still_requires_controller_evidence(rig):
    g, n, a, m = rig
    p, _, expired = boundary(*rig)
    a.latest = None; a.acceptances.clear()
    calls=[]
    def sleep(seconds):
        g.clock.sleep(seconds); calls.append(seconds)
        if len(calls)==3: emit(a, dict(expired, evaluated_ns=g.clock()))
    m.sleep = sleep
    assert m._ack(p)==(True, 'command_deadline_expired')
    assert len(calls)==3


@pytest.mark.parametrize('field', ['issued_ns', 'sequence', 'generation', 'bridge_epoch', 'controller_session', 'host_boot_id', 'deadline_ns'])
def test_acknowledgement_identity_is_exact(rig, field):
    g, n, a, m = rig
    p, _, _ = boundary(*rig)
    p[field] = p[field]+1 if type(p[field]) is int else 'different-owner'
    ok, _ = m._ack(p)
    assert not ok


def test_lost_acknowledgement_remains_uncertain_no_redispatch(rig):
    g, n, a, m = rig
    original=n.publish
    def lose(msg):
        if msg.operation==HEARTBEAT: return
        original(msg)
    n.publish=lose
    r=m.execute(.1,0,0,1.)
    assert r['state']=='uncertain' and not r['completed_progress_credit']
    assert r['controller_gait_zero'] and m.latched
    starts=[p for p in n.messages if p['operation']==START]
    assert len(starts)==1
    assert m.execute(.1,0,0,.1)['state']=='refused'


def test_stop_before_acknowledgement_cannot_resume_or_receive_credit(rig):
    g,n,a,m=rig
    def cancel(p):
        if p['operation']==START:
            n.hook=None
            assert m.emergency_stop()['ok']
    n.hook=cancel
    r=m.execute(.1,0,0,.5)
    assert r['state']=='interrupted' and not r['completed_progress_credit']
    assert r['controller_acceptance']['last_request_accepted'] is True
    assert not any(p['operation']==HEARTBEAT for p in n.messages)
    assert not g.tick()['active']


def test_short_command_expiry_during_start_ack_completes_with_exact_zero(rig):
    g,n,a,m=rig
    orig=m._ack
    def slow_ack(p,**kw):
        if p['operation']==START:
            g.clock.sleep(.101);n.emit()
        return orig(p,**kw)
    m._ack=slow_ack
    r=m.execute(.1,0,0,.1)
    assert r['state']=='completed' and r['reason']=='bounded_deadline_completed'
    assert r['controller_acceptance']['last_request_accepted']
    assert r['transport_attempted'] and r['controller_gait_zero']
    assert r['completed_progress_credit']
    assert r['stop_certificate']['last_request_issued_ns'] >= r['deadline_ns']
    assert r['stop_certificate']['last_request_accepted'] and r['stop_certificate']['gait_zero']
    assert len([p for p in n.messages if p['operation']==START])==1
    assert not any(p['operation']==HEARTBEAT for p in n.messages)


@pytest.mark.parametrize('outcome', ['receipt', 'emergency'])
def test_accepted_command_then_interruption_never_gets_progress(rig,outcome):
    g,n,a,m=rig
    orig=m._ack
    def slow_ack(p,**kw):
        if p['operation']==START:
            if outcome=='receipt':g.clock.sleep(.251);n.emit()
            else:g.line('Z '+str(g.clock()));n.emit()
        return orig(p,**kw)
    m._ack=slow_ack
    r=m.execute(.1,0,0,1.)
    assert r['state']=='interrupted' and r['controller_gait_zero']
    assert not r['completed_progress_credit']
    assert r['controller_acceptance']['last_request_accepted']


def test_delayed_active_status_cannot_resurrect_deadline_expired_state(rig):
    g,n,a,m=rig
    p,active,expired=boundary(*rig)
    emit(a,active)
    assert a.snapshot()[0]['reason']=='command_deadline_expired'
    assert m._ack(p)==(True,'command_deadline_expired')


def test_duplicate_status_does_not_refresh_age(rig):
    g,n,a,m=rig
    p,_,expired=boundary(*rig)
    before=a.received
    g.clock.sleep(.001);emit(a,expired)
    assert a.received==before
    g.clock.sleep(.251)
    assert a.acknowledgement(p) is None
    assert not m._ack(p)[0]


def test_controller_session_restart_clears_acceptance_and_rejects_old_status(rig):
    g,n,a,m=rig
    p,_,old=boundary(*rig)
    new=dict(old,controller_session='replacement-controller',last_request_accepted=False,active=False)
    emit(a,new)
    assert not a.acceptances
    emit(a,dict(old,evaluated_ns=g.clock()+1))
    assert a.snapshot()[0]['controller_session']=='replacement-controller'
    assert m._ack(p)==(False,'controller_restarted')


def test_successor_owner_cannot_be_authorized_by_retained_old_acceptance(rig):
    g,n,a,m=rig
    p,_,expired=boundary(*rig)
    # A later controller-issued command belongs to another epoch/generation.
    later=dict(expired,bridge_epoch='successor-epoch',generation=2,sequence=1,
               last_operation=START,last_request_generation=2,last_request_sequence=1,
               last_request_issued_ns=g.clock()+1,evaluated_ns=g.clock()+1,active=True,gait_zero=False)
    g.clock.sleep(.000000001);emit(a,later)
    assert m._ack(p)==(False,'controller_command_owner_changed')


def test_bridge_restart_epoch_cannot_use_previous_acknowledgement(rig):
    g,n,a,m=rig
    p,_,_=boundary(*rig)
    restarted=HeartbeatCommands(a,'host-1',clock=g.clock,sleep=g.clock.sleep,epoch='new-bridge')
    restarted.session=g.session
    assert not restarted._ack(dict(p,bridge_epoch='new-bridge'))[0]


def test_retention_is_bounded_and_eviction_never_infers_acceptance(rig):
    g,n,a,m=rig
    p,_,expired=boundary(*rig)
    for i in range(1,20):
        g.clock.sleep(.000000001)
        emit(a,dict(expired,evaluated_ns=g.clock(),last_request_sequence=2+i,sequence=2+i,
                    last_request_issued_ns=g.clock()))
    assert len(a.acceptances)==8
    assert a.acknowledgement(p) is None
    assert not m._ack(p)[0]


def test_session_retention_overflow_fails_closed(rig):
    g,n,a,m=rig
    p,_,expired=boundary(*rig)
    for i in range(65):emit(a,dict(expired,controller_session='restart-'+str(i)))
    assert a.status_history_fault and a.snapshot()[0] is None
    assert not a.acceptances and not m._ack(p)[0]


def test_covered_tail_preserves_original_deadline_and_receipt_expiry(rig):
    g,n,a,m=rig
    r=m.execute(.1,0,0,2.)
    assert r['state']=='completed' and r['controller_gait_zero']
    pp=[p for p in n.messages if p['operation'] in (START,HEARTBEAT)]
    assert len({p['deadline_ns'] for p in pp})==1
    assert pp[0]['deadline_ns']-pp[0]['start_ns']==2_000_000_000
    assert pp[-1]['deadline_ns']-pp[-1]['issued_ns'] < TIMEOUT_NS
    assert n.messages[-1]['operation']==STOP


def test_equal_time_conflicting_active_status_cannot_resurrect_authority(rig):
    g,n,a,m=rig
    p,active,expired=boundary(*rig)
    emit(a,dict(active,evaluated_ns=expired['evaluated_ns']))
    assert a.snapshot()[0]['active'] is False
    assert m._ack(p)==(True,'command_deadline_expired')


def test_old_ack_does_not_mutate_successor_command_evidence(rig):
    g,n,a,m=rig
    p,_,expired=boundary(*rig)
    m.current={'generation':2,'retired':False}
    m._ack(p)
    assert 'acceptance' not in m.current
