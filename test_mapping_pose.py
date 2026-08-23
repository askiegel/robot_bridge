#!/usr/bin/env python3

from types import SimpleNamespace

import app as bridge

from mapping_pose import MappingPoseProvider
from tf2_ros import TransformException


class FakeNow:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeClock:
    def __init__(self, nanoseconds):
        self._nanoseconds = nanoseconds

    def now(self):
        return FakeNow(self._nanoseconds)


class FakeNode:
    def __init__(self, nanoseconds):
        self._clock = FakeClock(nanoseconds)

    def get_clock(self):
        return self._clock


class FakeBuffer:
    def __init__(self, transform=None, error=None):
        self.transform = transform
        self.error = error
        self.calls = []

    def lookup_transform(
        self,
        target,
        source,
        when,
        timeout=None,
    ):
        self.calls.append(
            (target, source, when, timeout)
        )

        if self.error is not None:
            raise self.error

        return self.transform


def make_transform():
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=100,
                nanosec=250_000_000,
            ),
        ),
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=0.619351,
                y=0.069412,
                z=0.0,
            ),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=-0.021726,
                w=0.999764,
            ),
        ),
    )


def running_mapping():
    return {
        "running": True,
        "owned": True,
        "pid": 59303,
        "state": "RUNNING",
    }


def test_mapping_pose_uses_map_to_base_link():
    provider = MappingPoseProvider(
        FakeNode(100_500_000_000),
        FakeBuffer(transform=make_transform()),
    )

    snapshot = provider.snapshot()

    assert snapshot["available"] is True
    assert snapshot["status"] == "READY"
    assert snapshot["source"] == "cartographer_tf"

    assert snapshot["age_seconds"] == 0.25

    pose = snapshot["pose"]

    assert pose["frame_id"] == "map"
    assert pose["source_frame_id"] == "base_link"
    assert pose["position"]["x"] == 0.619351
    assert pose["position"]["y"] == 0.069412
    assert pose["position"]["z"] == 0.0

    assert pose["orientation"]["z"] == -0.021726
    assert pose["orientation"]["w"] == 0.999764


def test_mapping_pose_reports_missing_tf():
    provider = MappingPoseProvider(
        FakeNode(100_500_000_000),
        FakeBuffer(
            error=TransformException(
                "map to base_link unavailable"
            )
        ),
    )

    snapshot = provider.snapshot()

    assert snapshot["available"] is False
    assert snapshot["status"] == "TF_UNAVAILABLE"
    assert snapshot["pose"] is None


def test_mapping_pose_endpoint_requires_mapping(
    monkeypatch,
):
    monkeypatch.setattr(
        bridge.mapping_control,
        "snapshot",
        lambda: {
            "running": False,
            "owned": False,
            "state": "STOPPED",
        },
    )

    response = bridge.app.test_client().get(
        "/telemetry/mapping-pose"
    )

    payload = response.get_json()

    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["runtime_active"] is False
    assert payload["telemetry"]["status"] == (
        "MAPPING_STOPPED"
    )


def test_mapping_pose_endpoint_exposes_fresh_pose(
    monkeypatch,
):
    class FakePublisher:
        @staticmethod
        def mapping_pose_snapshot():
            return {
                "available": True,
                "status": "READY",
                "source": "cartographer_tf",
                "age_seconds": 0.05,
                "error": None,
                "pose": {
                    "frame_id": "map",
                    "source_frame_id": "base_link",
                    "position": {
                        "x": 0.6,
                        "y": 0.1,
                        "z": 0.0,
                    },
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0,
                    },
                },
            }

    monkeypatch.setattr(
        bridge.mapping_control,
        "snapshot",
        running_mapping,
    )
    monkeypatch.setattr(
        bridge,
        "ros_ready",
        True,
    )
    monkeypatch.setattr(
        bridge,
        "publisher_node",
        FakePublisher(),
    )

    response = bridge.app.test_client().get(
        "/telemetry/mapping-pose"
    )

    payload = response.get_json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["runtime_active"] is True
    assert payload["read_only"] is True

    telemetry = payload["telemetry"]

    assert telemetry["available"] is True
    assert telemetry["pose"]["frame_id"] == "map"
    assert (
        telemetry["pose"]["source_frame_id"]
        == "base_link"
    )


def test_mapping_pose_endpoint_is_get_only():
    assert bridge.app.test_client().post(
        "/telemetry/mapping-pose"
    ).status_code == 405
