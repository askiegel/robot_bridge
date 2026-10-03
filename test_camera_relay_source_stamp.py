from types import SimpleNamespace

import camera_relay


def _frame(jpeg, sec, nanosec):
    return SimpleNamespace(
        data=jpeg,
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
    )


def test_latest_jpeg_response_keeps_its_ros_stamp_pair():
    with camera_relay.lock:
        camera_relay.latest_jpeg = None
        camera_relay.latest_timestamp = None
        camera_relay.latest_source_stamp = None
        camera_relay.latest_frame_monotonic = None

    node = camera_relay.CameraRelayNode.__new__(camera_relay.CameraRelayNode)
    client = camera_relay.app.test_client()

    node.image_callback(_frame(b"jpeg-a", 12, 34))
    response_a = client.get("/camera/latest.jpg")
    assert response_a.status_code == 200
    assert response_a.mimetype == "image/jpeg"
    assert response_a.data == b"jpeg-a"
    assert response_a.headers["X-Mayday-Source-Stamp-Sec"] == "12"
    assert response_a.headers["X-Mayday-Source-Stamp-Nanosec"] == "34"
    assert response_a.headers["X-Mayday-Source-Stamp-Ns"] == str(
        12 * 1_000_000_000 + 34
    )

    node.image_callback(_frame(b"jpeg-b", 13, 56))
    response_b = client.get("/camera/latest.jpg")
    assert response_b.status_code == 200
    assert response_b.mimetype == "image/jpeg"
    assert response_b.data == b"jpeg-b"
    assert response_b.headers["X-Mayday-Source-Stamp-Sec"] == "13"
    assert response_b.headers["X-Mayday-Source-Stamp-Nanosec"] == "56"
    assert response_b.headers["X-Mayday-Source-Stamp-Ns"] == str(
        13 * 1_000_000_000 + 56
    )
