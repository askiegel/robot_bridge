import math

FORWARD_SPEED = 0.10
MAX_DURATION_SECONDS = 0.50
PUBLISH_HZ = 20.0
SECTOR_HALF_ANGLE = math.radians(30.0)
MAX_SCAN_AGE_SECONDS = 0.30
START_CLEARANCE_M = 0.45
STOP_CLEARANCE_M = 0.35
MIN_VALID_SAMPLES = 5
LIDAR_FRAME = "lidar_link"
ROTATION_TO_ROBOT_RADIANS = math.pi / 2.0
MAYDAY_SELF_BEARING_MIN_DEG = -15.0
MAYDAY_SELF_BEARING_MAX_DEG = -2.0
MAYDAY_SELF_MAX_RANGE_M = 0.15


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def is_mayday_self_return(bearing, distance):
    degrees = math.degrees(bearing)
    return MAYDAY_SELF_BEARING_MIN_DEG <= degrees < MAYDAY_SELF_BEARING_MAX_DEG and distance <= MAYDAY_SELF_MAX_RANGE_M


def forward_clearance(snapshot):
    if not snapshot.get('available'):
        return {'ok': False, 'reason': 'lidar_unavailable', 'age_seconds': snapshot.get('age_seconds'), 'sample_count': 0, 'clearance_m': None}
    age = snapshot.get('age_seconds')
    if age is None or age > MAX_SCAN_AGE_SECONDS:
        return {'ok': False, 'reason': 'lidar_stale', 'age_seconds': age, 'sample_count': 0, 'clearance_m': None}
    scan = snapshot.get('scan') or {}
    frame_id = scan.get('frame_id')
    if frame_id != LIDAR_FRAME:
        return {'ok': False, 'reason': 'unsupported_lidar_frame', 'frame_id': frame_id, 'age_seconds': age, 'sample_count': 0, 'self_return_filtered_count': 0, 'clearance_m': None}
    valid=[]
    filtered=0
    for index, value in enumerate(scan.get('ranges') or []):
        if value is None or not math.isfinite(value) or value < scan.get('range_min', 0.0) or value > scan.get('range_max', 0.0):
            continue
        raw_angle=scan.get('angle_min', 0.0)+index*scan.get('angle_increment', 0.0)
        bearing=normalize_angle(raw_angle + ROTATION_TO_ROBOT_RADIANS)
        if is_mayday_self_return(bearing, value):
            filtered += 1
            continue
        if abs(bearing) <= SECTOR_HALF_ANGLE:
            valid.append(value)
    if len(valid) < MIN_VALID_SAMPLES:
        return {'ok': False, 'reason': 'insufficient_forward_samples', 'frame_id': frame_id, 'rotation_to_robot_radians': ROTATION_TO_ROBOT_RADIANS, 'age_seconds': age, 'sample_count': len(valid), 'self_return_filtered_count': filtered, 'clearance_m': None}
    return {'ok': True, 'reason': None, 'frame_id': frame_id, 'rotation_to_robot_radians': ROTATION_TO_ROBOT_RADIANS, 'age_seconds': age, 'sample_count': len(valid), 'self_return_filtered_count': filtered, 'clearance_m': min(valid)}


def start_allowed(gate):
    return gate['ok'] and gate['clearance_m'] >= START_CLEARANCE_M


def continue_allowed(gate):
    return gate['ok'] and gate['clearance_m'] > STOP_CLEARANCE_M


# Same calibrated base_link transform and total radius as Cognitive's
# local_motion_safety_envelope. This gate is additive: it does not replace JIT.
BASE_X_M = -0.078701
BASE_Y_M = 0.000495
PROTECTED_RADIUS_M = 0.45
MAX_SCAN_SAMPLES = 8192


def protected_sweep(points, sector_counts, axes, remaining_seconds):
    """Current-frame full capsule (translation) or circle (pure rotation).

    Unlike the legacy directional probe, no starting-circle or side return is
    exempt. Rotation retains the circular envelope, not only the inner 0.22 m
    footprint. Remaining command travel bounds a sweep; it is not displacement.
    Full-circle coverage is required because the starting footprint is protected.
    """
    result = dict(permitted=False, reason='active_lidar_geometry_invalid',
                  protected_radius_m=PROTECTED_RADIUS_M)
    if (len(axes) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in axes)
            or sum(v != 0 for v in axes) != 1
            or type(remaining_seconds) not in (int, float)
            or not math.isfinite(remaining_seconds) or not 0 <= remaining_seconds <= 2):
        return result
    if (len(sector_counts) != 8 or any(type(n) is not int or n < MIN_VALID_SAMPLES
                                     for n in sector_counts)):
        return dict(result, reason='active_lidar_coverage_missing')
    ex, ey = axes[0] * remaining_seconds, axes[1] * remaining_seconds
    length2 = ex * ex + ey * ey
    nearest = None
    for x, y in points:
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            return result
        t = max(0., min(1., (x * ex + y * ey) / length2)) if length2 else 0.
        gap = math.hypot(x - t * ex, y - t * ey)
        if nearest is None or gap < nearest[0]:
            nearest = gap, x, y
    if nearest is None:
        return result
    result.update(minimum_swept_clearance_m=nearest[0], nearest_x_m=nearest[1],
                  nearest_y_m=nearest[2], remaining_seconds=remaining_seconds)
    return dict(result, permitted=nearest[0] > PROTECTED_RADIUS_M,
                reason='active_lidar_clear' if nearest[0] > PROTECTED_RADIUS_M
                       else 'active_lidar_intrusion')


def bounded_command_clearance(snapshot, axes, remaining_seconds):
    """Pure cached scan adapter. No acquisition, raw output, or motion writer."""
    result = dict(permitted=False, reason='active_lidar_unavailable')
    if not isinstance(snapshot, dict) or snapshot.get('available') is not True:
        if isinstance(snapshot, dict) and snapshot.get('status') == 'CACHE_BUSY':
            result['reason'] = 'active_lidar_cache_busy'
        return result
    if snapshot.get('currentness_fault'):
        return dict(result, reason=snapshot['currentness_fault'])
    age = snapshot.get('age_seconds')
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= MAX_SCAN_AGE_SECONDS:
        return dict(result, reason='active_lidar_stale')
    scan = snapshot.get('scan')
    if not isinstance(scan, dict) or scan.get('frame_id') != LIDAR_FRAME:
        return dict(result, reason='active_lidar_frame_invalid')
    metadata = [scan.get(k) for k in ('angle_min', 'angle_increment', 'range_min', 'range_max')]
    ranges = scan.get('ranges')
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in metadata)
            or metadata[1] == 0 or metadata[2] < 0 or metadata[3] <= metadata[2]
            or not isinstance(ranges, list) or not 1 <= len(ranges) <= MAX_SCAN_SAMPLES):
        return dict(result, reason='active_lidar_geometry_invalid')
    points, counts = [], [0] * 8
    for i, distance in enumerate(ranges):
        if type(distance) not in (int, float) or not math.isfinite(distance) or not metadata[2] <= distance <= metadata[3]:
            continue
        bearing = normalize_angle(metadata[0] + i * metadata[1] + ROTATION_TO_ROBOT_RADIANS)
        if is_mayday_self_return(bearing, distance):
            continue
        x = BASE_X_M + distance * math.cos(bearing)
        y = BASE_Y_M + distance * math.sin(bearing)
        points.append((x, y))
        # Octants in base-frame bearing, including wrapped rear.
        counts[int(((math.atan2(y, x) + math.pi / 8) % (2 * math.pi)) / (math.pi / 4)) % 8] += 1
    return protected_sweep(points, counts, axes, remaining_seconds)
