from pathlib import Path


def node_source():
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
    text = node_source()

    start = text.index(
        "    def navigation_start_preflight("
    )

    end = text.index(
        "    def initialize_planning_localization(",
        start,
    )

    return text[
        start:end
    ]


def test_no_continuous_navigation_tf_objects():
    text = node_source()

    for symbol in (
        "navigation_preflight_tf",
        "navigation_tf_history",
        "navigation_tf_publisher",
        "navigation_tf_publish_timer",
        "navigation_odom_subscription",
        "navigation_static_tf_subscription",
    ):
        assert symbol not in text


def test_normal_telemetry_remains():
    text = node_source()

    assert "'/scan'," in text
    assert "'/odom/local'," in text


def test_transient_listener_is_started_before_candidate_scans():
    text = preflight_source()

    session = text.index(
        "with self.navigation_tf_lookup.session() "
        "as tf_lookup:"
    )

    listener = text.index(
        "listener_started_at = time.monotonic()"
    )

    candidate = text.index(
        "candidate_stamp ="
    )

    assert session < listener < candidate


def test_preflight_advances_through_new_scans():
    text = preflight_source()

    assert (
        "last_attempted_scan_received_at"
        in text
    )

    assert (
        "candidate_received_at"
        in text
    )

    assert (
        "> listener_started_at"
        in text
    )

    assert (
        "> last_attempted_scan_received_at"
        in text
    )


def test_scan_is_not_accepted_until_exact_time_lookup_succeeds():
    text = preflight_source()

    lookup = text.index(
        "tf_lookup.lookup_transform("
    )

    accepted = text.index(
        "scan_stamp = candidate_stamp"
    )

    assert lookup < accepted

    assert (
        "except TransformException as exc:"
        in text
    )

    assert (
        "continue"
        in text[
            lookup:accepted
        ]
    )


def test_exact_scan_timestamp_semantics_are_preserved():
    text = preflight_source()

    assert (
        "Time.from_msg("
        in text
    )

    assert (
        "candidate_stamp"
        in text
    )

    assert (
        "'odom',"
        in text
    )

    assert (
        "candidate_frame"
        in text
    )


def test_preflight_has_bounded_capture_window():
    text = preflight_source()

    assert (
        "capture_deadline"
        in text
    )

    assert (
        "max("
        in text
    )

    assert (
        "3.0"
        in text
    )


def test_preflight_reports_no_exact_time_candidate():
    text = preflight_source()

    assert (
        "No fresh /scan message had an "
        in text
    )

    assert (
        "'exact-time transform '"
        in text
    )

    assert (
        "'transient TF capture window.'"
        in text
    )
