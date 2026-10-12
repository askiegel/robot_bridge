"""Real C++ watchdog + real generated ROS messages; simulated ROS transport only."""
import json
import math
import os
import pathlib
import subprocess
import threading
import time

import pytest
from champ_heartbeat import HeartbeatCommands, SYNC, START, HEARTBEAT, STOP, EMERGENCY
from champ_ros_adapter import ChampTransport, GuardedCommandStatus
from rclpy.serialization import serialize_message, deserialize_message
from champ_msgs.msg import GuardedCommand

BASE = 1_000_000_000_000_000_123

class Clock:
    def __init__(self): self.now = BASE
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += round(seconds*1e9)

class Gate:
    def __init__(self, clock, session='controller-1'):
        self.clock, self.session = clock, session
        self.lock = threading.Lock()
        self.process = subprocess.Popen([os.environ['CHAMP_GUARD_CLI'], session],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.commands = []
        self.last = self.tick()
    def close(self):
        self.process.stdin.close(); self.process.wait(timeout=2)
    def line(self, text):
        with self.lock:
            self.process.stdin.write(text+'\n'); self.process.stdin.flush()
            self.last = json.loads(self.process.stdout.readline()); return self.last
    def tick(self): return self.line('T '+str(self.clock()))
    def packet(self, operation, **kwargs):
        p=dict(operation=operation, controller_session=self.session, host_boot_id='host-1',
               bridge_epoch='bridge-1', generation=0, sequence=0, issued_ns=self.clock(),
               start_ns=0, deadline_ns=0, linear_x=0., linear_y=0., angular_z=0.)
        p.update(kwargs); return p
    def send(self, packet):
        self.commands.append(dict(packet))
        fields=['operation','controller_session','host_boot_id','bridge_epoch','generation',
                'sequence','issued_ns','start_ns','deadline_ns','linear_x','linear_y','angular_z']
        return self.line('P '+str(self.clock())+' '+' '.join(str(packet[k]) for k in fields))
    def sync(self, **kwargs):
        r=self.send(self.packet(SYNC, **kwargs));self.clock.sleep(.000000001);return r
    def start(self, **kwargs):
        p=self.packet(START,generation=1,sequence=1,start_ns=self.clock(),
                      deadline_ns=self.clock()+1_000_000_000,linear_x=.1)
        p.update(kwargs); self.start_packet=p; return self.send(p)
    def hb(self, **kwargs):
        p=dict(self.start_packet,operation=HEARTBEAT,sequence=self.last['sequence']+1,issued_ns=self.clock())
        p.update(kwargs);return self.send(p)
    def stop(self):
        p=dict(self.start_packet,operation=STOP,sequence=self.last['sequence']+1,
               issued_ns=self.clock(),linear_x=0.,linear_y=0.,angular_z=0.)
        self.stop_packet=p;return self.send(p)

@pytest.fixture
def gate():
    g=Gate(Clock());yield g;g.close()

@pytest.mark.parametrize('axes',[(.1,0.,0.),(-.1,0.,0.),(0.,.08,0.),(0.,-.08,0.),(0.,0.,.3),(0.,0.,-.3)])
def test_valid_advancing_commands_preserve_velocity_and_full_two_seconds(gate,axes):
    assert gate.sync()['accepted']
    assert gate.start(linear_x=axes[0],linear_y=axes[1],angular_z=axes[2],
                      deadline_ns=gate.clock()+2_000_000_000)['accepted']
    for _ in range(39):
        gate.clock.sleep(.05);s=gate.hb();assert s['accepted'] and s['active']
        assert (s['linear_x'],s['linear_y'],s['angular_z'])==axes
    gate.clock.sleep(.05);s=gate.tick();assert not s['active']
    assert s['reason']=='command_deadline_expired'

@pytest.mark.parametrize('fault',['silence','disconnect','equal_sequence','reordered_sequence','replayed_start'])
def test_no_receipt_refresh_from_frozen_or_duplicate_input(gate,fault):
    gate.sync();gate.start();gate.clock.sleep(.1)
    if fault=='equal_sequence':assert not gate.hb(sequence=1)['accepted']
    elif fault=='reordered_sequence':assert not gate.hb(sequence=0)['accepted']
    elif fault=='replayed_start':assert not gate.send(gate.start_packet)['accepted']
    gate.clock.sleep(.15);assert not gate.tick()['active']
    assert gate.last['reason'] in ('command_receipt_expired','invalid_command_or_deadline')

@pytest.mark.parametrize('mutation',[
    {'deadline_ns':BASE+1_500_000_000},{'start_ns':BASE+2},
    {'linear_x':.2},{'linear_y':.08},{'issued_ns':BASE},
])
def test_heartbeat_cannot_mutate_or_extend_command(gate,mutation):
    gate.sync();gate.start();gate.clock.sleep(.05)
    s=gate.hb(**mutation);assert not s['accepted'] and not s['active']

@pytest.mark.parametrize('kind',['heartbeat','start','stop'])
def test_delayed_duplicates_after_stop_cannot_rearm(gate,kind):
    gate.sync();gate.start();old=gate.start_packet;gate.clock.sleep(.05)
    assert gate.stop()['accepted']
    p=dict(old,operation=HEARTBEAT,sequence=2) if kind=='heartbeat' else old if kind=='start' else gate.stop_packet
    s=gate.send(p);assert not s['accepted'] and not s['active']

@pytest.mark.parametrize('field,value',[
    ('controller_session','old-controller'),('host_boot_id','old-host'),('bridge_epoch','wrong-owner'),
    ('issued_ns',BASE+1000),('issued_ns',BASE-250_000_000),('deadline_ns',BASE),
    ('deadline_ns',BASE+2_000_000_100),('linear_x',float('nan')),('linear_x',.5001),
    ('linear_y',.10001),('angular_z',1.0001),('sequence',2),('generation',0)])
def test_bad_admission_never_grants_velocity(gate,field,value):
    gate.sync();s=gate.start(**{field:value});assert not s['accepted'] and not s['active']

@pytest.mark.parametrize('zero_kind',['scoped','emergency','legacy'])
def test_explicit_zero_retires_generation(gate,zero_kind):
    gate.sync();gate.start();gate.clock.sleep(.05)
    if zero_kind=='scoped':s=gate.stop()
    elif zero_kind=='emergency':s=gate.send(gate.packet(EMERGENCY,generation=1))
    else:s=gate.line('Z '+str(gate.clock()))
    assert not s['active'];gate.clock.sleep(.01)
    assert not gate.hb(sequence=4)['accepted']
    assert gate.start(generation=2, start_ns=gate.clock(), issued_ns=gate.clock(),
                      deadline_ns=gate.clock()+500_000_000)['accepted']


def test_busy_refusal_is_single_use_and_does_not_change_owner(gate):
    gate.sync();gate.start();p=dict(gate.start_packet,generation=2)
    s=gate.send(p);assert not s['accepted'] and s['generation']==1 and s['active']
    gate.clock.sleep(.05);gate.stop();assert not gate.send(p)['accepted']


def test_bridge_restart_requires_zero_and_retires_old_epoch(gate):
    gate.sync();gate.start();assert not gate.sync(bridge_epoch='new-bridge')['accepted']
    gate.clock.sleep(.25);assert not gate.tick()['active']
    assert gate.sync(bridge_epoch='new-bridge')['accepted']
    assert not gate.sync(bridge_epoch='bridge-1')['accepted']


def test_controller_restart_invalidates_prior_session(gate):
    gate.sync();gate.start();old=gate.start_packet
    replacement=Gate(gate.clock,session='controller-2')
    try:
        assert not replacement.send(old)['accepted'];assert not replacement.last['active']
    finally: replacement.close()


def test_watchdog_ignores_ros_time_and_clock_regression_is_latched(gate):
    gate.sync();gate.start();gate.clock.now-=1
    assert not gate.tick()['active']
    gate.clock.now+=100;assert not gate.sync()['accepted']


def test_bounded_epoch_history_fails_closed(gate):
    gate.sync()
    for n in range(64): assert gate.sync(bridge_epoch='epoch-'+str(n))['accepted']
    assert not gate.sync(bridge_epoch='overflow')['accepted']

class SimNode:
    """No ROS participant. Publish is typed serialization + real C++ gate."""
    def __init__(self, gate): self.gate=gate;self.callback=None;self.messages=[];self.hook=None;self.fail=None
    def create_publisher(self, *args): return self
    def create_subscription(self, typ, topic, callback, depth): self.callback=callback;return object()
    def emit(self):
        s=self.gate.tick();m=GuardedCommandStatus()
        for field in m.get_fields_and_field_types():
            if field!='velocity':setattr(m,field,s[field])
        m.velocity.linear.x=float(s['linear_x']);m.velocity.linear.y=float(s['linear_y']);m.velocity.angular.z=float(s['angular_z'])
        self.callback(m)
    def publish(self,msg):
        # Exercise real generated message serialization/type support.
        msg=deserialize_message(serialize_message(msg),GuardedCommand)
        p={k:getattr(msg,k) for k in ['operation','controller_session','host_boot_id','bridge_epoch',
                                     'generation','sequence','issued_ns','start_ns','deadline_ns']}
        p.update(linear_x=msg.velocity.linear.x,linear_y=msg.velocity.linear.y,angular_z=msg.velocity.angular.z)
        self.messages.append(p);self.gate.send(p);self.emit()
        if self.hook:self.hook(p)
        if self.fail and self.fail(p):raise RuntimeError('simulated publication uncertainty')

@pytest.fixture
def rig(gate):
    node=SimNode(gate);adapter=ChampTransport(node,lambda:None,clock=gate.clock)
    node.emit();orig=adapter.snapshot
    def snapshot():node.emit();return orig()
    adapter.snapshot=snapshot
    manager=HeartbeatCommands(adapter,'host-1',clock=gate.clock,sleep=gate.clock.sleep,epoch='bridge-1')
    return gate,node,adapter,manager

@pytest.mark.parametrize('axes',[(.1,0,0),(-.1,0,0),(0,.08,0),(0,-.08,0),(0,0,.3),(0,0,-.3)])
def test_real_adapter_and_controller_component_bounded_two_second_command(rig,axes):
    g,n,a,m=rig;r=m.execute(*axes,2.)
    assert r['state']=='completed' and r['controller_gait_zero'] and r['completed_progress_credit']
    packets=[p for p in n.messages if p['operation'] in (START,HEARTBEAT)]
    assert len(packets)==40 and len({p['deadline_ns'] for p in packets})==1
    assert packets[-1]['issued_ns']<r['deadline_ns']
    assert all(type(p['issued_ns']) is int and p['issued_ns']>2**53 for p in packets)
    assert n.messages[-1]['operation']==STOP and not g.last['active']


def test_emergency_cancellation_never_gets_completed_credit_or_late_heartbeat(rig):
    g,n,a,m=rig
    def hook(p):
        if p['operation']==HEARTBEAT and p['sequence']==4:
            n.hook=None;assert m.emergency_stop()['ok']
    n.hook=hook;r=m.execute(.1,0,0,2.)
    assert r['state']=='interrupted' and not r['completed_progress_credit']
    stop_at=next(i for i,p in enumerate(n.messages) if p['operation']==EMERGENCY)
    assert not any(p['operation'] in (START,HEARTBEAT) for p in n.messages[stop_at+1:])
    old=[p for p in n.messages if p['operation']==HEARTBEAT][-1]
    assert not g.send(old)['accepted']
    assert m.execute(0,.08,0,.1)['state']=='completed'

@pytest.mark.parametrize('fail_operation',[START,HEARTBEAT,STOP])
def test_uncertain_publication_and_failed_stop_prevent_replacement(rig,fail_operation):
    g,n,a,m=rig;n.fail=lambda p:p['operation']==fail_operation
    r=m.execute(.1,0,0,.1);assert r['state']=='uncertain' and m.latched
    assert not r['completed_progress_credit']
    assert m.execute(.1,0,0,.1)['state']=='refused'

@pytest.mark.parametrize('values',[(0,0,0,.1),(.1,.1,0,.1),(.1,0,.1,.1),(.1,0,0,2.001),(.1,0,0,0),(math.nan,0,0,.1)])
def test_unsupported_commands_are_certified_zero_transport(rig,values):
    g,n,a,m=rig;r=m.execute(*values);assert r['state']=='refused'
    assert not r['transport_attempted'] and n.messages==[]


def test_competing_command_and_completion_stop_cannot_affect_successor(rig):
    g,n,a,m=rig;results=[]
    def hook(p):
        if p['operation']==START:results.append(m.execute(0,.08,0,.1))
    n.hook=hook;r=m.execute(.1,0,0,.1)
    assert results[0]['state']=='refused' and not results[0]['transport_attempted']
    assert r['state']=='completed'
    oldstop=n.messages[-1];n.hook=None
    assert m.execute(0,.08,0,.1)['state']=='completed'
    assert not g.send(oldstop)['accepted']


def test_stopped_controller_restart_requires_new_sync(rig):
    g,n,a,m=rig;assert m.execute(.1,0,0,.1)['state']=='completed'
    old=g.start_packet if hasattr(g,'start_packet') else n.messages[1]
    g.close();new=Gate(g.clock,session='controller-2');n.gate=new
    try:
        assert m.execute(0,.08,0,.1)['state']=='completed'
        assert m.session=='controller-2';assert not new.send(old)['accepted']
    finally:
        new.close();g.close=lambda:None


def test_active_controller_restart_is_uncertain_and_never_auto_resumed(rig):
    g,n,a,m=rig;replacement=[]
    def hook(p):
        if p['operation']==HEARTBEAT:
            n.hook=None;g.close();new=Gate(g.clock,session='controller-2');n.gate=new;replacement.append(new)
    n.hook=hook
    try:
        r=m.execute(.1,0,0,1.);assert r['state']=='uncertain' and m.latched
        assert not r['completed_progress_credit'];assert not replacement[0].tick()['active']
    finally:
        for new in replacement:new.close()
        g.close=lambda:None

@pytest.mark.parametrize('ingress',['body_pose','legacy_twist','unused_axis'])
def test_competing_ingress_does_not_renew_or_replace_velocity_authority(gate,ingress):
    gate.sync();gate.start();gate.clock.sleep(.2)
    if ingress=='body_pose':s=gate.line('B '+str(gate.clock()))
    else:s=gate.line('L '+str(gate.clock())+' .2 0 0 '+('0' if ingress=='unused_axis' else '1'))
    assert s['active'] and s['linear_x']==.1
    gate.clock.sleep(.05);assert not gate.tick()['active']
    assert gate.last['reason']=='command_receipt_expired'


def test_expiry_is_irrevocable_even_for_advancing_packet(gate):
    gate.sync();gate.start();gate.clock.sleep(.25);assert not gate.tick()['active']
    assert not gate.hb(sequence=100)['accepted']
    assert not gate.start(generation=1)['accepted']


def test_clock_delay_reports_receipt_loss_before_later_deadline(gate):
    gate.sync();gate.start();gate.clock.sleep(2.)
    assert gate.tick()['reason']=='command_receipt_expired'


def test_scheduler_stall_does_not_receive_completed_progress_credit(rig):
    g,n,a,m=rig
    def stalled_sleep(seconds):g.clock.sleep(2.)
    m.sleep=stalled_sleep
    r=m.execute(.1,0,0,1.)
    assert r['state']=='interrupted' and not r['completed_progress_credit']
    assert r['controller_gait_zero'] and r['transport_attempted']


def test_cancellation_during_completion_stop_cannot_receive_completed_credit(rig):
    g,n,a,m=rig
    def cancel_at_stop(p):
        if p['operation']==STOP:
            n.hook=None;assert m.emergency_stop()['ok']
    n.hook=cancel_at_stop;r=m.execute(.1,0,0,.1)
    assert r['state'] in ('interrupted','uncertain') and not r['completed_progress_credit']
    assert not g.tick()['active']


def test_old_completion_cannot_stop_or_clear_replacement_owner(rig):
    g,n,a,m=rig;old=[]
    def save(p):
        if p['operation']==START:old.append(m.current)
    n.hook=save;assert m.execute(.1,0,0,.1)['state']=='completed'
    def late(p):
        if p['operation']==START:
            current=m.current;count=len(n.messages)
            assert m._stop(old[0])[0]
            m._finish(old[0],'completed','old_completion',True)
            assert m.current is current and len(n.messages)==count and g.last['active']
    n.hook=late;assert m.execute(0,.08,0,.1)['state']=='completed'


def test_missing_and_frozen_controller_status_is_bounded_zero_transport(rig):
    g,n,a,m=rig;a.snapshot=lambda:(None,0)
    r=m.execute(.1,0,0,.1)
    assert r['state']=='refused' and not r['transport_attempted'] and not n.messages


def test_lost_heartbeat_ack_retires_command_and_latches_uncertainty(rig):
    g,n,a,m=rig;original=n.publish
    def drop(msg):
        if msg.operation==HEARTBEAT:
            # ROS publication returned, but controller acceptance is unknown.
            return
        original(msg)
    n.publish=drop;a.publisher=n
    r=m.execute(.1,0,0,1.)
    assert r['state']=='uncertain' and not r['completed_progress_credit']
    assert r['controller_gait_zero'] and not g.tick()['active']
    assert m.execute(.1,0,0,.1)['state']=='refused'


def test_frozen_clock_ack_loop_remains_finite(rig):
    g,n,a,m=rig;n.publish=lambda _:None
    m.sleep=lambda _:None
    assert m.execute(.1,0,0,.1)['state']=='refused'


def test_real_threads_dispatch_cancel_and_competing_admission_are_fenced(rig):
    g,n,a,m=rig;entered=threading.Event();release=threading.Event();cancelled=threading.Event()
    answers={};original_sleep=m.sleep
    def held_publish(p):
        if p['operation']==START:
            entered.set();assert release.wait(2.)
    def await_cancel(seconds):
        if threading.current_thread().name=='bounded-command' and entered.is_set():
            assert cancelled.wait(2.)
        original_sleep(seconds)
    n.hook=held_publish;m.sleep=await_cancel
    runner=threading.Thread(name='bounded-command',target=lambda:answers.update(command=m.execute(.1,0,0,1.)))
    def cancel():
        answers['cancel']=m.emergency_stop();cancelled.set()
    runner.start();assert entered.wait(2.)
    stopper=threading.Thread(target=cancel);stopper.start();release.set()
    runner.join(3.);stopper.join(3.)
    assert not runner.is_alive() and not stopper.is_alive()
    assert answers['cancel']['ok'] and answers['command']['state']=='interrupted'
    assert not answers['command']['completed_progress_credit']
    emergency_index=next(i for i,p in enumerate(n.messages) if p['operation']==EMERGENCY)
    assert all(p['operation'] not in (START,HEARTBEAT) for p in n.messages[emergency_index+1:])
    assert not g.tick()['active']

@pytest.mark.parametrize('zero',['legacy','emergency','scoped'])
def test_delayed_start_not_previously_seen_cannot_rearm_after_zero(gate,zero):
    gate.sync()
    queued=gate.packet(START,generation=20,sequence=1,start_ns=gate.clock(),
                       deadline_ns=gate.clock()+1_000_000_000,linear_x=.1)
    gate.clock.sleep(.01)
    if zero=='legacy':gate.line('Z '+str(gate.clock()))
    elif zero=='emergency':gate.send(gate.packet(EMERGENCY))
    else:gate.send(gate.packet(STOP,generation=1,sequence=1))
    s=gate.send(queued);assert not s['accepted'] and not s['active']
    assert s['reason']=='command_predates_revocation'
    gate.clock.sleep(.001)
    assert gate.start(generation=21)['accepted']


def test_positive_regressed_sequence_cannot_refresh_receipt(gate):
    gate.sync();gate.start();gate.clock.sleep(.05);assert gate.hb(sequence=3)['accepted']
    gate.clock.sleep(.2);s=gate.hb(sequence=2)
    assert not s['accepted'] and s['active'] and s['sequence']==3
    gate.clock.sleep(.05);assert not gate.tick()['active']


def test_refused_competing_generation_stop_cannot_stop_current_owner(gate):
    gate.sync();gate.start();assert not gate.start(generation=2)['accepted']
    s=gate.send(gate.packet(STOP,generation=2,sequence=2))
    assert not s['accepted'] and s['active'] and s['generation']==1
    assert not gate.send(gate.packet(STOP,generation=1,sequence=0))['accepted']


def test_start_equal_to_zero_timestamp_is_ambiguous_and_refused(gate):
    gate.sync();gate.line('Z '+str(gate.clock()))
    assert not gate.start()['accepted']
    gate.clock.sleep(.000000001);assert gate.start(generation=2)['accepted']
