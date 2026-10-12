"""ROS message adapter; construction alone does not start ROS or publish."""
from collections import OrderedDict
import threading
import time
from champ_msgs.msg import GuardedCommand, GuardedCommandStatus


class ChampTransport:
    def __init__(self, node, emergency_zero, *, clock=time.monotonic_ns, on_status=None):
        self.on_status = on_status
        self.clock = clock
        self.emergency_zero = emergency_zero
        self.lock = threading.Lock()
        self.latest = None
        self.received = 0
        # Single-flight commands need very little history. This stores only
        # controller-issued acceptance, never publication-derived evidence.
        self.acceptances = OrderedDict()
        self.retired_sessions = set()
        self.status_history_fault = False
        self.publisher = node.create_publisher(GuardedCommand, '/guarded_command', 1)
        self.subscription = node.create_subscription(
            GuardedCommandStatus, '/guarded_command/status', self.receive, 1)

    def receive(self, msg):
        fields = ['protocol_version', 'controller_session', 'host_boot_id', 'bridge_epoch',
                  'evaluated_ns', 'generation', 'sequence', 'synchronized', 'active',
                  'gait_zero', 'deadline_ns', 'reason', 'last_operation',
                  'last_request_generation', 'last_request_sequence',
                  'last_request_issued_ns', 'last_request_accepted']
        state = {k: getattr(msg, k) for k in fields}
        state.update(linear_x=msg.velocity.linear.x, linear_y=msg.velocity.linear.y,
                     angular_z=msg.velocity.angular.z)
        with self.lock:
            if self.status_history_fault:
                return
            if self.latest:
                previous = self.latest
                if state['controller_session'] != previous['controller_session']:
                    if state['controller_session'] in self.retired_sessions:
                        return
                    if len(self.retired_sessions) >= 64:
                        self.status_history_fault = True
                        self.latest = None
                        self.acceptances.clear()
                        return
                    self.retired_sessions.add(previous['controller_session'])
                    self.acceptances.clear()
                elif state['evaluated_ns'] < previous['evaluated_ns']:
                    return  # A delayed packet must not resurrect active state.
                elif (state['evaluated_ns'] == previous['evaluated_ns']
                      and self._ack_key(state, status=True) == self._ack_key(previous, status=True)
                      and previous['active'] is False and state['active'] is True):
                    return  # Equal-time ambiguity cannot resurrect authority.
                elif state == previous:
                    return  # Duplicate delivery cannot refresh evidence age.
            self.latest, self.received = state, self.clock()
            if state['last_request_accepted'] is True:
                key = self._ack_key(state, status=True)
                self.acceptances[key] = dict(state)
                self.acceptances.move_to_end(key)
                while len(self.acceptances) > 8:
                    self.acceptances.popitem(last=False)
        if self.on_status:
            self.on_status(state)

    @staticmethod
    def _ack_key(value, *, status=False):
        fields = ('controller_session', 'host_boot_id', 'bridge_epoch')
        request = ('last_operation', 'last_request_generation',
                   'last_request_sequence', 'last_request_issued_ns') if status else (
                   'operation', 'generation', 'sequence', 'issued_ns')
        return tuple(value[k] for k in fields + request)

    def acknowledgement(self, packet):
        with self.lock:
            state = self.acceptances.get(self._ack_key(packet))
            if not state or not self.latest:
                return None
            now = self.clock()
            if (state['controller_session'] != self.latest['controller_session']
                    or not 0 <= now - state['evaluated_ns'] < 250_000_000):
                return None
            return dict(state)

    def snapshot(self):
        with self.lock:
            return (dict(self.latest) if self.latest else None), self.received

    def publish(self, packet):
        msg = GuardedCommand()
        for field in ['operation', 'controller_session', 'host_boot_id', 'bridge_epoch',
                      'generation', 'sequence', 'issued_ns', 'start_ns', 'deadline_ns']:
            setattr(msg, field, packet[field])
        msg.velocity.linear.x = float(packet['linear_x'])
        msg.velocity.linear.y = float(packet['linear_y'])
        msg.velocity.angular.z = float(packet['angular_z'])
        self.publisher.publish(msg)
