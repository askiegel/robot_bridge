"""ROS message adapter; construction alone does not start ROS or publish."""
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
            self.latest, self.received = state, self.clock()
        if self.on_status:
            self.on_status(state)

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
