import math

FORWARD_SPEED = 0.10
MAX_DURATION_SECONDS = 0.50
PUBLISH_HZ = 20.0
SECTOR_HALF_ANGLE = math.radians(30.0)
MAX_SCAN_AGE_SECONDS = 0.30
START_CLEARANCE_M = 0.45
STOP_CLEARANCE_M = 0.35
MIN_VALID_SAMPLES = 5


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def forward_clearance(snapshot):
    if not snapshot.get('available'):
        return {'ok': False, 'reason': 'lidar_unavailable', 'age_seconds': snapshot.get('age_seconds'), 'sample_count': 0, 'clearance_m': None}
    age = snapshot.get('age_seconds')
    if age is None or age > MAX_SCAN_AGE_SECONDS:
        return {'ok': False, 'reason': 'lidar_stale', 'age_seconds': age, 'sample_count': 0, 'clearance_m': None}
    scan = snapshot.get('scan') or {}
    valid=[]
    for index, value in enumerate(scan.get('ranges') or []):
        if value is None or not math.isfinite(value) or value < scan.get('range_min', 0.0) or value > scan.get('range_max', 0.0):
            continue
        angle=normalize_angle(scan.get('angle_min', 0.0)+index*scan.get('angle_increment', 0.0))
        if abs(angle) <= SECTOR_HALF_ANGLE:
            valid.append(value)
    if len(valid) < MIN_VALID_SAMPLES:
        return {'ok': False, 'reason': 'insufficient_forward_samples', 'age_seconds': age, 'sample_count': len(valid), 'clearance_m': None}
    return {'ok': True, 'reason': None, 'age_seconds': age, 'sample_count': len(valid), 'clearance_m': min(valid)}


def start_allowed(gate):
    return gate['ok'] and gate['clearance_m'] >= START_CLEARANCE_M


def continue_allowed(gate):
    return gate['ok'] and gate['clearance_m'] > STOP_CLEARANCE_M
