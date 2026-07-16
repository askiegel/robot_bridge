#!/usr/bin/env python3

import time
from unittest.mock import patch

import app as bridge


class FakePublisher:
    def __init__(self):
        self.commands = []

    def publish_motion(
        self,
        linear_x,
        angular_z,
    ):
        self.commands.append(
            (
                float(linear_x),
                float(angular_z),
                time.monotonic(),
            )
        )


def reset_state():
    with bridge.motion_lock:
        bridge.motion_state.update(
            {
                "streaming": False,
                "linear_x": 0.0,
                "angular_z": 0.0,
                "deadline_monotonic": None,
                "last_command_at": None,
                "last_stop_at": None,
                "watchdog_stop_count": 0,
            }
        )


def main():
    fake_publisher = FakePublisher()

    bridge.ros_ready = True
    bridge.publisher_node = fake_publisher
    bridge.ros_error = None

    reset_state()

    client = bridge.app.test_client()

    # Warm up Flask's test client so one-time request setup is not included
    # in the streaming endpoint response-time measurement.
    warmup = client.get("/status")
    assert warmup.status_code == 200

    print("===== STREAMING COMMAND RETURNS IMMEDIATELY =====")

    started = time.monotonic()

    response = client.post(
        "/motion",
        json={
            "linear_x": 0.0,
            "angular_z": 0.5,
            "duration": 0.25,
            "streaming": True,
            "watchdog_timeout": 0.25,
        },
    )

    elapsed = time.monotonic() - started
    payload = response.get_json()

    print(response.status_code, payload)

    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["mode"] == "streaming"
    assert payload["returned_immediately"] is True

    print(
        f"Streaming endpoint response time: "
        f"{elapsed:.4f} seconds"
    )

    # The endpoint must return well before the 0.25-second watchdog period.
    # A generous offline threshold avoids failures caused by test-runner
    # scheduling while still detecting the old blocking implementation.
    assert elapsed < 0.20

    with bridge.motion_lock:
        assert (
            bridge.motion_state["streaming"]
            is True
        )

        assert (
            bridge.motion_state["angular_z"]
            == 0.5
        )

    print()
    print("===== STREAMING COMMAND REFRESHES WATCHDOG =====")

    first_deadline = (
        bridge.motion_state[
            "deadline_monotonic"
        ]
    )

    time.sleep(0.03)

    response = client.post(
        "/motion",
        json={
            "linear_x": 0.08,
            "angular_z": 0.1,
            "duration": 0.25,
            "streaming": True,
            "watchdog_timeout": 0.25,
        },
    )

    assert response.status_code == 200

    with bridge.motion_lock:
        assert (
            bridge.motion_state[
                "deadline_monotonic"
            ]
            > first_deadline
        )

        assert (
            bridge.motion_state["linear_x"]
            == 0.08
        )

        assert (
            bridge.motion_state["angular_z"]
            == 0.1
        )

    print()
    print("===== STOP CANCELS STREAMING =====")

    response = client.post("/stop")
    payload = response.get_json()

    print(response.status_code, payload)

    assert response.status_code == 200
    assert payload["ok"] is True

    with bridge.motion_lock:
        assert (
            bridge.motion_state["streaming"]
            is False
        )

    assert fake_publisher.commands[-1][0:2] == (
        0.0,
        0.0,
    )

    print()
    print("===== LEGACY BOUNDED MOTION REMAINS AVAILABLE =====")

    with patch(
        "app.time.sleep",
        return_value=None,
    ):
        response = client.post(
            "/motion",
            json={
                "linear_x": 0.05,
                "angular_z": 0.0,
                "duration": 0.10,
            },
        )

    payload = response.get_json()

    print(response.status_code, payload)

    assert response.status_code == 200
    assert payload["mode"] == "bounded"
    assert payload["automatic_stop"] is True
    assert (
        payload["returned_immediately"]
        is False
    )

    print()
    print("PASS: streaming request returns immediately")
    print("PASS: streaming request refreshes watchdog")
    print("PASS: STOP immediately cancels streaming")
    print("PASS: bounded compatibility mode remains")
    print()
    print("Robot Bridge streaming tests passed.")


if __name__ == "__main__":
    main()
