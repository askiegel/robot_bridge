"""One bounded command, fenced publications, and confirmed controller gait-zero.

No ROS/network I/O at import. No durable journal, reservation, or physical-stop
claim. The transport supplies latest real controller status and ROS publication.
"""
import math
import threading
import time
import uuid
from local_motion_guard import bounded_command_clearance, MAX_SCAN_AGE_SECONDS

HEARTBEAT_NS = 50_000_000   # Offline timing target, not measured stopping latency.
TIMEOUT_NS = 250_000_000
SAFETY_POLL_NS = 50_000_000  # Existing interlock cadence; not a latency guarantee.
MAX_DURATION_NS = 2_000_000_000
SYNC, START, HEARTBEAT, STOP, EMERGENCY = range(5)


class HeartbeatCommands:
    def __init__(self, transport, host_boot_id, *, clock=time.monotonic_ns,
                 sleep=time.sleep, epoch=None, lidar_reader=None):
        self.transport, self.host = transport, host_boot_id
        self.clock, self.sleep = clock, sleep
        self.epoch = epoch or str(uuid.uuid4())
        self.lock = threading.RLock()
        self.current = None
        self.generation = 0
        self.session = None
        self.latched = False
        self.lidar_reader = lidar_reader

    def _lidar_decision(self, axes, remaining, previous=None):
        """Read only local cached telemetry; return bounded current-frame facts."""
        started = self.clock()
        try:
            sample = self.lidar_reader() if callable(self.lidar_reader) else None
        except Exception:
            sample = None
        try:
            result = bounded_command_clearance(sample, axes, remaining)
        except Exception:
            result = dict(permitted=False, reason='active_lidar_geometry_invalid')
        if not isinstance(sample, dict) or sample.get('available') is not True:
            return result
        session, seq, stamp, receipt = (sample.get(k) for k in
            ('producer_session', 'acquisition_sequence', 'source_stamp_ns', 'received_monotonic_seconds'))
        if (type(session) is not str or not 1 <= len(session) <= 64
                or type(seq) is not int or not 0 < seq < 2**64
                or type(stamp) is not int or not 0 <= stamp < 2**64
                or type(receipt) not in (int, float) or not math.isfinite(receipt)
                or not 0 <= receipt <= self.clock() / 1e9):
            return dict(permitted=False, reason='active_lidar_identity_invalid')
        # Include evaluation/reader residence. No network or source-clock
        # subtraction; source stamps identify scans, local receipt measures age.
        now = self.clock()
        age = sample.get('age_seconds')
        if type(age) not in (int, float) or not math.isfinite(age):
            return dict(permitted=False, reason='active_lidar_stale')
        age = max(age + (now - started) / 1e9, now / 1e9 - receipt)
        result.update(producer_session=session, acquisition_sequence=seq,
                      source_stamp_ns=stamp, received_monotonic_seconds=receipt,
                      effective_age_seconds=age, evaluated_ns=now)
        if sample.get('currentness_fault') in {'active_lidar_source_regressed',
                'active_lidar_source_stamp_invalid', 'active_lidar_source_stamp_reused'}:
            result['source_fault'] = sample['currentness_fault']
        if age < 0 or age > MAX_SCAN_AGE_SECONDS or now < started:
            result.update(permitted=False, reason='active_lidar_stale')
        if previous:
            if session != previous['producer_session']:
                result.update(permitted=False, reason='active_lidar_session_changed')
            elif (seq < previous['acquisition_sequence'] or stamp < previous['source_stamp_ns']
                    or (seq == previous['acquisition_sequence']) != (stamp == previous['source_stamp_ns'])):
                result.update(permitted=False, reason='active_lidar_source_regressed')
            elif stamp == previous['source_stamp_ns'] and receipt != previous['received_monotonic_seconds']:
                result.update(permitted=False, reason='active_lidar_frozen_rejuvenation')
        return result

    def _check_safety(self, c):
        # Synchronous with the owned execute/ack loop, not an asynchronous STOP
        # worker. An old generation cannot check or revoke a successor.
        with self.lock:
            if self.current is not c or c['retired']:
                return False
            if self.clock() < c['next_safety_check']:
                return True
            gate = self._lidar_decision(c['axes'], max(0., (c['end'] - self.clock()) / 1e9),
                                        c['lidar_safety'])
            if self.current is not c or c['retired']:
                return False  # Reader/cancellation cannot restore authority.
            c['lidar_safety'] = gate
            c['next_safety_check'] = self.clock() + SAFETY_POLL_NS
            if gate['permitted'] is not True:
                c['safety_failure'] = gate['reason']
                return False
            return True

    def _status(self):
        s, received = self.transport.snapshot()
        now = self.clock()
        if not isinstance(s, dict) or type(received) is not int or not 0 <= now-received < TIMEOUT_NS:
            return None
        if (s.get('protocol_version') != 1 or s.get('host_boot_id') != self.host
                or not isinstance(s.get('controller_session'), str)
                or not 1 <= len(s['controller_session']) <= 64
                or type(s.get('evaluated_ns')) is not int
                or not 0 <= now-s['evaluated_ns'] < TIMEOUT_NS):
            return None
        return s

    def _packet(self, operation, c=None):
        now = self.clock()
        return dict(operation=operation, controller_session=self.session,
                    host_boot_id=self.host, bridge_epoch=self.epoch,
                    generation=c['generation'] if c else 0,
                    sequence=c['sequence'] if c else 0, issued_ns=now,
                    start_ns=c['start'] if c else 0,
                    deadline_ns=c['end'] if c else 0,
                    linear_x=c['axes'][0] if c and operation in (START, HEARTBEAT) else 0.,
                    linear_y=c['axes'][1] if c and operation in (START, HEARTBEAT) else 0.,
                    angular_z=c['axes'][2] if c and operation in (START, HEARTBEAT) else 0.)

    def _publish(self, p):
        # Called only under the generation fence; no deferred writer queue.
        self.transport.publish(p)

    @staticmethod
    def _zero(s):
        return (s and s.get('active') is False and s.get('gait_zero') is True
                and all(type(s.get(k)) in (int, float) and s[k] == 0
                        for k in ('linear_x', 'linear_y', 'angular_z')))

    def _ack(self, p, *, zero=False):
        limit = self.clock() + TIMEOUT_NS
        for _ in range(50):
            if self.clock() >= limit:
                break
            s = self._status()
            if s and s['controller_session'] != p['controller_session']:
                return False, 'controller_restarted'
            accepted = None if zero or not s else self.transport.acknowledgement(p)
            accepted_exact = (accepted and accepted.get('generation') == p['generation']
                              and accepted.get('sequence') == p['sequence']
                              and accepted.get('deadline_ns') == p['deadline_ns'])
            # Record acceptance as a historical fact even when cancellation has
            # won. It cannot restore active authority or completed progress.
            with self.lock:
                if (accepted_exact and self.current
                        and self.current['generation'] == p['generation']
                        and p['bridge_epoch'] == self.epoch
                        and p['controller_session'] == self.session):
                    self.current['acceptance'] = {
                        k: accepted[k] for k in (
                            'controller_session', 'bridge_epoch', 'last_operation',
                            'last_request_generation', 'last_request_sequence',
                            'last_request_issued_ns', 'last_request_accepted', 'evaluated_ns')}
                if p['operation'] in (START, HEARTBEAT) and self.current and self.current['retired']:
                    return False, 'command_cancelled'
                if (not zero and p['operation'] in (START, HEARTBEAT) and self.current
                        and self.current['generation'] == p['generation']
                        and p['bridge_epoch'] == self.epoch
                        and p['controller_session'] == self.session
                        and not self._check_safety(self.current)):
                    return False, self.current.get('safety_failure') or 'command_cancelled'
            exact = (s and s.get('last_operation') == p['operation']
                     and s.get('last_request_generation') == p['generation']
                     and s.get('last_request_sequence') == p['sequence']
                     and s.get('last_request_issued_ns') == p['issued_ns'])
            if zero:
                if exact:
                    if s.get('last_request_accepted') is not True:
                        return False, s.get('reason', 'controller_refused')
                    if p['operation'] == STOP and (
                            s.get('bridge_epoch') != p['bridge_epoch']
                            or s.get('generation') != p['generation']
                            or s.get('sequence') != p['sequence']):
                        return False, 'controller_command_owner_changed'
                    if self._zero(s):
                        return True, s.get('reason')
            elif s:
                if accepted_exact:
                    # Acceptance and the latest outcome have separate meaning.
                    if (s.get('bridge_epoch') != p['bridge_epoch']
                            or s.get('generation') != p['generation']):
                        return False, 'controller_command_owner_changed'
                    if s.get('active') is not True:
                        return True, s.get('reason', 'controller_interrupted')
                    return True, 'controller_accepted'
                if exact and s.get('last_request_accepted') is not True:
                    return False, s.get('reason', 'controller_refused')
            self.sleep(.005)
        return False, 'controller_ack_timeout'

    def _synchronize(self):
        s = self._status()
        if not self._zero(s):
            return False, 'controller_not_current_and_zero'
        self.session = s['controller_session']
        p = self._packet(SYNC)
        try:
            self._publish(p)
        except Exception:
            return False, 'synchronization_publication_failed'
        limit = self.clock() + TIMEOUT_NS
        for _ in range(50):
            if self.clock() >= limit:
                break
            s = self._status()
            if (s and s['controller_session'] == self.session and s.get('bridge_epoch') == self.epoch
                    and s.get('synchronized') is True and self._zero(s) and s.get('last_operation') == SYNC
                    and s.get('last_request_issued_ns') == p['issued_ns']
                    and s.get('last_request_accepted') is True):
                return True, 'synchronized'
            self.sleep(.005)
        return False, 'synchronization_refused_or_timeout'

    def execute(self, x, y, yaw, duration):
        vals = (x, y, yaw, duration)
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in vals)
                or not 0 < duration <= 2 or abs(x) > .5 or abs(y) > .1 or abs(yaw) > 1
                or sum(v != 0 for v in (x, y, yaw)) != 1):
            return self._result('refused', 'unsupported_bounded_command', False, False)
        with self.lock:
            if self.current or self.latched:
                return self._result('refused', 'owner_busy_or_unresolved', False, False)
            gate = self._lidar_decision((x, y, yaw), duration)
            if gate['permitted'] is not True:
                return dict(self._result('refused', gate['reason'], False, False), lidar_safety=gate)
            s = self._status()
            if not (s and s['controller_session'] == self.session and s.get('bridge_epoch') == self.epoch
                    and self._zero(s)):
                ok, why = self._synchronize()
                if not ok:
                    return self._result('refused', why, False, False)
            # A START must be strictly later than the controller's last zero
            # evaluation. Equal-clock timestamps are ambiguous and fail closed.
            s = self._status()
            if not s:
                return self._result('refused', 'controller_status_not_current', False, False)
            if self.clock() <= s['evaluated_ns']:
                self.sleep(.005)
            now = self.clock()
            if now <= s['evaluated_ns']:
                return self._result('refused', 'start_clock_not_after_zero', False, False)
            self.generation += 1
            c = dict(generation=self.generation, sequence=1, start=now,
                     end=now+int(duration*1e9), axes=(x,y,yaw), retired=False, cancelled=False, last_send=now,
                     stop_packet=None, stop_result=None, lidar_safety=gate,
                     next_safety_check=0, safety_failure=None)
            self.current = c
            if not self._check_safety(c):
                self.current = None  # START was never published; no scoped STOP.
                return dict(self._result('refused', c['safety_failure'] or 'command_cancelled', False, False),
                            lidar_safety=c['lidar_safety'])
            p = self._packet(START, c)
            attempted = True
            try:
                self._publish(p)
            except Exception:
                return self._finish(c, 'uncertain', 'start_publication_uncertain', attempted)
        ok, why = self._ack(p)
        if not ok:
            return self._finish(c, 'interrupted' if c['retired'] or
                (why == c['safety_failure'] and c.get('acceptance')) else 'uncertain', why, attempted)
        if why != 'controller_accepted':
            return self._finish(c, 'interrupted', why, attempted)
        next_send = p['issued_ns'] + HEARTBEAT_NS
        while self.clock() < c['end']:
            with self.lock:
                if c['retired']:
                    return self._finish(c, 'interrupted', 'command_cancelled', attempted)
                if not self._check_safety(c):
                    return self._finish(c, 'interrupted', c['safety_failure'] or 'command_cancelled', attempted)
                s = self._status()
                if not s or s['controller_session'] != self.session:
                    return self._finish(c, 'uncertain', 'controller_status_lost_or_restarted', attempted)
                if (s.get('active') is not True or s.get('generation') != c['generation']
                        or s.get('bridge_epoch') != self.epoch):
                    return self._finish(c, 'interrupted', s.get('reason', 'controller_interrupted'), attempted)
                now = self.clock()
                if now >= c['end']:
                    break
                # Avoid a needless last-tail heartbeat: the already accepted
                # receipt covers this tail under the unchanged expiry bound.
                # This neither extends nor truncates the immutable command end.
                covered_tail = (c['end'] - now < HEARTBEAT_NS
                                and c['end'] - c['last_send'] < TIMEOUT_NS)
                if now >= next_send and not covered_tail:
                    # Status/ack work may have crossed the next sensing check.
                    # Never renew authority using a now-overdue safety result.
                    if not self._check_safety(c):
                        return self._finish(c, 'interrupted', c['safety_failure'] or 'command_cancelled', attempted)
                    c['sequence'] += 1
                    p = self._packet(HEARTBEAT, c)
                    try:
                        self._publish(p)
                    except Exception:
                        return self._finish(c, 'uncertain', 'heartbeat_publication_uncertain', attempted)
                    c['last_send'] = p['issued_ns']
                    next_send = p['issued_ns'] + HEARTBEAT_NS
                else:
                    p = None
            if p:
                ok, why = self._ack(p)
                if not ok:
                    return self._finish(c, 'interrupted' if c['retired'] or
                        (why == c['safety_failure'] and c.get('acceptance')) else 'uncertain', why, attempted)
            if p and why != 'controller_accepted':
                return self._finish(c, 'interrupted', why, attempted)
            self.sleep(min(.005, max(0, (c['end']-self.clock())/1e9)))
        with self.lock:
            s = self._status()
            if not s or s['controller_session'] != self.session:
                return self._finish(c, 'uncertain', 'completion_status_unverified', attempted)
            if (c['end']-c['last_send'] >= TIMEOUT_NS or s.get('generation') != c['generation']
                    or s.get('bridge_epoch') != self.epoch
                    or (s.get('active') is not True and s.get('reason') != 'command_deadline_expired')):
                return self._finish(c, 'interrupted', s.get('reason', 'completion_interrupted'), attempted)
        return self._finish(c, 'completed', 'bounded_deadline_completed', attempted)

    def _stop(self, c):
        with self.lock:
            if self.current is not c:
                # A retained acknowledgement is historical only. No old STOP
                # publication or current-owner mutation is allowed.
                return c.get('stop_result') or (False, 'retired_command_owner')
            # Retire before any zero publication; old execute loops are fenced.
            c['retired'] = True
            if c['stop_result'] is not None:
                return c['stop_result']
            if c['stop_packet'] is None:
                c['sequence'] += 1
                c['stop_packet'] = self._packet(STOP, c)
                try:
                    self._publish(c['stop_packet'])
                except Exception:
                    c['stop_result'] = (False, 'stop_publication_uncertain')
                    self.latched = True
                    return c['stop_result']
            p = c['stop_packet']
        result = self._ack(p, zero=True)
        with self.lock:
            c['stop_result'] = result
            if not result[0]:
                self.latched = True
        return result

    def _finish(self, c, state, reason, attempted):
        # RLock callers are safe; current remains held through STOP resolution.
        with self.lock:
            if self.current is not c:
                return self._result('refused', 'retired_command_owner', False, False)
            if c['cancelled']:
                state, reason = 'interrupted', 'command_cancelled'
        ok, stop_reason = self._stop(c)
        with self.lock:
            # Cancellation may arrive during the STOP acknowledgment wait.
            if c['cancelled']:
                state, reason = 'interrupted', 'command_cancelled'
            if state == 'uncertain' or not ok:
                self.latched = True
                state = 'uncertain'
            if self.current is c:
                self.current = None
        r = self._result(state, reason, attempted, ok)
        r.update(generation=c['generation'], start_ns=c['start'], deadline_ns=c['end'],
                 controller_session=self.session, bridge_epoch=self.epoch,
                 stop_reason=stop_reason,
                 controller_acceptance=c.get('acceptance'), lidar_safety=c['lidar_safety'])
        return r

    def emergency_stop(self):
        with self.lock:
            c = self.current
            if c:
                c['retired'] = c['cancelled'] = True
            s = self._status()
            if not s:
                self.latched = True
                try:
                    self.transport.emergency_zero()  # Legacy zero is stop-only.
                except Exception:
                    pass
                return dict(ok=False, reason='unverified_emergency_zero')
            self.session = s['controller_session']
            p = self._packet(EMERGENCY, c)
            try:
                self._publish(p)
            except Exception:
                self.latched = True
                try:
                    self.transport.emergency_zero()
                except Exception:
                    pass
                return dict(ok=False, reason='emergency_publication_uncertain')
            if c:
                c['stop_packet'] = p
        ok, why = self._ack(p, zero=True)
        with self.lock:
            if c:
                c['stop_result'] = (ok, why)
            if not ok:
                self.latched = True
        return dict(ok=ok, reason=why, controller_gait_zero=ok)

    @staticmethod
    def _result(state, reason, attempted, zero):
        return dict(ok=state == 'completed' and zero, state=state, reason=reason,
                    transport_attempted=attempted, controller_gait_zero=zero,
                    completed_progress_credit=state == 'completed' and zero,
                    physical_stop_verified=False)
