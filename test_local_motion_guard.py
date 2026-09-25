import math
from local_motion_guard import continue_allowed, forward_clearance, start_allowed


def snapshot(ranges, age=0.0, available=True):
    return {"available": available, "age_seconds": age, "scan": {"angle_min": -math.pi / 2 - 0.4, "angle_increment": 0.1, "frame_id": "lidar_link", "range_min": 0.1, "range_max": 5.0, "ranges": ranges}} if available else {"available": False, "age_seconds": None}


def test_forward_sector_filters_ranges_and_thresholds():
    ranges = [float("nan"), float("inf"), -float("inf"), 0.01, 0.8, 1.0, 0.45, 0.5, 0.6, 0.7, 0.8, 1.0, 1.0]
    gate = forward_clearance(snapshot(ranges))
    assert gate["ok"] and gate["sample_count"] >= 5
    assert gate["clearance_m"] == 0.45 and start_allowed(gate)
    blocked = dict(gate, clearance_m=0.44)
    assert not start_allowed(blocked)
    assert not continue_allowed(dict(gate, clearance_m=0.35))
    assert continue_allowed(dict(gate, clearance_m=0.36))


def test_stale_unavailable_and_insufficient_fail_closed():
    assert forward_clearance(snapshot([1.0] * 13, age=0.31))["reason"] == "lidar_stale"
    assert forward_clearance(snapshot([], available=False))["reason"] == "lidar_unavailable"
    assert forward_clearance(snapshot([1.0] * 4))["reason"] == "insufficient_forward_samples"
