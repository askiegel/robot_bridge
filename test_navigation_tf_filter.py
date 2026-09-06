from pathlib import Path


def robot_bridge_source():
    source = Path(
        "app.py"
    ).read_text(
        encoding="utf-8"
    )

    return source[
        source.index(
            "class RobotBridgePublisher"
        ):
        source.index(
            "def ros_spin():"
        )
    ]


def preflight_source():
    source = robot_bridge_source()

    start = source.index(
        "    def navigation_start_preflight("
    )

    end = source.index(
        "    def initialize_planning_localization(",
        start,
    )

    return source[
        start:end
    ]


def test_navigation_tf_relay_is_not_permanent():
    source = robot_bridge_source()

    assert (
        "navigation_tf_publisher"
        not in source
    )

    assert (
        "navigation_tf_history"
        not in source
    )

    assert (
        "navigation_tf_publish_timer"
        not in source
    )

    assert (
        "navigation_preflight_tf"
        not in source
    )


def test_normal_scan_and_local_odom_telemetry_remain():
    source = robot_bridge_source()

    assert "'/scan'," in source
    assert "'/odom/local'," in source

    assert (
        "self.latest_scan_stamp"
        in source
    )

    assert (
        "self.latest_local_odom_received_at"
        in source
    )


def test_preflight_starts_tf_before_selecting_fresh_scan():
    source = preflight_source()

    session = source.index(
        "with self.navigation_tf_lookup.session() "
        "as tf_lookup:"
    )

    listener_time = source.index(
        "listener_started_at = time.monotonic()"
    )

    fresh_check = source.index(
        "> listener_started_at"
    )

    lookup = source.index(
        "tf_lookup.lookup_transform("
    )

    assert session < listener_time
    assert listener_time < fresh_check
    assert fresh_check < lookup


def test_preflight_keeps_exact_scan_timestamp_semantics():
    source = preflight_source()

    assert (
        "Time.from_msg("
        in source
    )

    assert (
        "scan_stamp"
        in source
    )

    assert (
        "tf_lookup.lookup_transform("
        in source
    )

    assert (
        "Time.from_msg("
        in source
    )

    lookup = source.index(
        "tf_lookup.lookup_transform("
    )

    accepted = source.index(
        "scan_stamp = candidate_stamp"
    )

    assert lookup < accepted
