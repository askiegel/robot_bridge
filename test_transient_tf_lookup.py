from pathlib import Path

import transient_tf_lookup


def helper_source():
    return Path(
        transient_tf_lookup.__file__
    ).read_text(
        encoding="utf-8"
    )


def robot_bridge_node_source():
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


def test_transient_lookup_has_session_api():
    source = helper_source()

    assert (
        "class TransientTfLookup"
        in source
    )

    assert (
        "def session(self):"
        in source
    )


def test_transient_listener_does_not_create_hidden_spin_thread():
    source = helper_source()

    assert (
        "spin_thread=False"
        in source
    )

    assert (
        "SingleThreadedExecutor"
        in source
    )

    assert (
        "threading.Thread("
        in source
    )


def test_transient_executor_has_explicit_shutdown():
    source = helper_source()

    assert (
        "executor.shutdown("
        in source
    )

    assert (
        "thread.join("
        in source
    )

    assert (
        "node.destroy_node()"
        in source
    )


def test_robot_bridge_has_no_permanent_dynamic_tf_subscription():
    source = robot_bridge_node_source()

    assert '"/tf",' not in source
    assert "'/tf'," not in source

    assert (
        "TransformListener("
        not in source
    )


def test_robot_bridge_has_no_permanent_static_tf_subscription():
    source = robot_bridge_node_source()

    assert '"/tf_static",' not in source
    assert "'/tf_static'," not in source


def test_robot_bridge_has_no_second_odom_subscription():
    source = robot_bridge_node_source()

    assert '"/odom",' not in source
    assert "'/odom'," not in source

    assert "'/odom/local'," in source


def test_mapping_pose_remains_demand_driven():
    source = robot_bridge_node_source()

    assert (
        "self.navigation_tf_lookup = "
        "TransientTfLookup()"
        in source
    )

    assert (
        "MappingPoseProvider("
        in source
    )
