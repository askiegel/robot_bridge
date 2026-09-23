import app as bridge


class FakeRosPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def reset_motion_audit():
    with bridge.motion_audit_lock:
        bridge.motion_audit["next_sequence"] = 1
        bridge.motion_audit["next_segment_id"] = 1
        bridge.motion_audit["active_segment"] = None
        bridge.motion_audit["recent_segments"].clear()
        bridge.motion_audit["recent_zero_events"].clear()


def setup_fake_egress(monkeypatch):
    raw_publisher = FakeRosPublisher()
    node = object.__new__(bridge.RobotBridgePublisher)
    node.publisher = raw_publisher

    monkeypatch.setattr(bridge, "ros_ready", True)
    monkeypatch.setattr(bridge, "ros_error", None)
    monkeypatch.setattr(bridge, "publisher_node", node)
    reset_motion_audit()

    return raw_publisher


def emit(linear_x, angular_z):
    result = bridge.publish_twist(linear_x, angular_z)
    assert result["ok"] is True


def test_motion_audit_endpoint_is_read_only(monkeypatch):
    raw_publisher = setup_fake_egress(monkeypatch)
    client = bridge.app.test_client()

    first = client.get("/motion/audit")
    second = client.get("/motion/audit")

    assert first.status_code == 200
    assert second.status_code == 200
    assert first.get_json() == second.get_json()
    assert first.get_json()["motion_audit"]["active_segment"] is None
    assert first.get_json()["motion_audit"]["recent_segments"] == []
    assert raw_publisher.messages == []


def test_forward_segment_is_completed_by_zero_egress(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.10, 0.0)
    emit(0.0, 0.0)

    audit = bridge.motion_audit_snapshot()
    assert audit["active_segment"] is None
    assert len(audit["recent_segments"]) == 1
    segment = audit["recent_segments"][0]
    assert segment["linear_x"] == 0.10
    assert segment["angular_z"] == 0.0
    assert segment["state"] == "completed"
    assert segment["publish_count"] == 1
    assert segment["elapsed_duration"] >= 0.0
    assert audit["recent_zero_events"][0]["event_type"] == "zero_egress"
    assert audit["recent_zero_events"][0]["previous_active_segment_id"] == segment["segment_id"]


def test_turn_segment_is_completed_by_zero_egress(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.0, -0.40)
    emit(0.0, 0.0)

    segment = bridge.motion_audit_snapshot()["recent_segments"][0]
    assert segment["linear_x"] == 0.0
    assert segment["angular_z"] == -0.40
    assert segment["state"] == "completed"


def test_identical_streaming_egress_stays_in_one_segment(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.08, 0.10)
    emit(0.08, 0.10)
    emit(0.08, 0.10)

    audit = bridge.motion_audit_snapshot()
    assert audit["recent_segments"] == []
    assert audit["active_segment"]["publish_count"] == 3
    assert audit["active_segment"]["last_publish_sequence"] == 3

    response = bridge.app.test_client().get("/motion/audit")
    assert response.status_code == 200
    assert response.get_json()["motion_audit"]["active_segment"]["publish_count"] == 3


def test_changed_nonzero_vector_replaces_active_segment(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.08, 0.0)
    emit(0.0, 0.30)

    audit = bridge.motion_audit_snapshot()
    completed = audit["recent_segments"]
    active = audit["active_segment"]
    assert len(completed) == 1
    assert completed[0]["linear_x"] == 0.08
    assert completed[0]["state"] == "completed"
    assert completed[0]["elapsed_duration"] >= 0.0
    assert active["linear_x"] == 0.0
    assert active["angular_z"] == 0.30
    assert active["state"] == "active"
    assert completed[0]["end_sequence"] == active["audit_sequence"]


def test_stop_robot_closes_active_segment_through_final_egress(monkeypatch):
    raw_publisher = setup_fake_egress(monkeypatch)
    monkeypatch.setattr(bridge.time, "sleep", lambda _: None)

    emit(0.05, 0.0)
    result = bridge.stop_robot()

    assert result["ok"] is True
    audit = bridge.motion_audit_snapshot()
    assert audit["active_segment"] is None
    assert len(audit["recent_segments"]) == 1
    assert audit["recent_zero_events"][-1]["previous_active_segment_id"] == 1
    assert raw_publisher.messages[-1].linear.x == 0.0
    assert raw_publisher.messages[-1].angular.z == 0.0


def test_watchdog_stop_uses_the_same_final_egress(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.05, 0.0)

    with bridge.motion_lock:
        bridge.motion_state.update(
            {
                "streaming": True,
                "linear_x": 0.05,
                "angular_z": 0.0,
                "deadline_monotonic": (
                    bridge.time.monotonic() - 1.0
                ),
                "watchdog_stop_count": 0,
            }
        )

    result = bridge.streaming_motion_step()

    assert result["ok"] is True
    audit = bridge.motion_audit_snapshot()
    assert audit["active_segment"] is None
    assert audit["recent_zero_events"][-1]["event_type"] == "zero_egress"
    assert audit["recent_zero_events"][-1]["previous_active_segment_id"] == 1
    with bridge.motion_lock:
        assert bridge.motion_state["watchdog_stop_count"] == 1


def test_motion_audit_sequences_are_monotonic(monkeypatch):
    setup_fake_egress(monkeypatch)

    emit(0.10, 0.0)
    emit(0.10, 0.0)
    emit(0.0, 0.20)
    emit(0.0, 0.0)

    audit = bridge.motion_audit_snapshot()
    first, second = audit["recent_segments"]
    zero_event = audit["recent_zero_events"][-1]
    assert first["audit_sequence"] == 1
    assert first["last_publish_sequence"] == 2
    assert second["audit_sequence"] == 3
    assert zero_event["sequence"] == 4
    assert audit["next_sequence"] == 5
