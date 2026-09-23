#!/usr/bin/env python3

from contextlib import contextmanager
import threading
import uuid

from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_msgs.msg import TFMessage


class _TransientTfSession:
    """
    Short-lived full-TF listener with explicitly owned executor.

    No /tf or /tf_static subscription exists outside the
    lifetime of this object.
    """

    def __init__(
        self,
        *,
        cache_seconds,
    ):
        self._closed = False

        self.node = Node(
            "transient_tf_lookup_"
            + uuid.uuid4().hex[:10]
        )

        self.buffer = Buffer(
            cache_time=Duration(
                seconds=float(cache_seconds)
            ),
            node=self.node,
        )

        # Do not let TransformListener create an executor/thread
        # internally. Own both here so shutdown is deterministic.
        self.listener = TransformListener(
            self.buffer,
            self.node,
            spin_thread=False,
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(
            self.node
        )

        self.thread = threading.Thread(
            target=self.executor.spin,
            name=(
                self.node.get_name()
                + "_executor"
            ),
            daemon=True,
        )

        self.thread.start()

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
        *,
        timeout,
    ):
        return self.buffer.lookup_transform(
            target_frame,
            source_frame,
            time,
            timeout=timeout,
        )

    def close(self):
        if self._closed:
            return

        self._closed = True

        executor = self.executor
        thread = self.thread
        listener = self.listener
        node = self.node

        self.executor = None
        self.thread = None
        self.listener = None
        self.node = None

        if executor is not None:
            executor.shutdown(
                timeout_sec=1.0
            )

        if (
            thread is not None
            and thread.is_alive()
        ):
            thread.join(
                timeout=1.0
            )

        if listener is not None:
            unregister = getattr(
                listener,
                "unregister",
                None,
            )

            if callable(unregister):
                unregister()

        if (
            executor is not None
            and node is not None
        ):
            try:
                executor.remove_node(
                    node
                )
            except Exception:
                pass

        if node is not None:
            node.destroy_node()


class TransientTfLookup:
    """
    Demand-driven TF lookup.

    Constructing this object creates no ROS node and no TF
    subscription. A session is created only for an explicit
    lookup/preflight operation.
    """

    def __init__(
        self,
        *,
        cache_seconds=2.0,
    ):
        cache_seconds = float(
            cache_seconds
        )

        if cache_seconds <= 0.0:
            raise ValueError(
                "cache_seconds must be positive"
            )

        self.cache_seconds = (
            cache_seconds
        )

    @contextmanager
    def session(self):
        session = _TransientTfSession(
            cache_seconds=(
                self.cache_seconds
            ),
        )

        try:
            yield session

        finally:
            session.close()

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
        *,
        timeout,
    ):
        with self.session() as session:
            return session.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=timeout,
            )


class _TransientOdometryTfSession:
    """Short-lived exact-time TF buffer for navigation startup."""

    _ODOM_PARENT_FRAME = "odom"
    _ODOM_CHILD_FRAME = "base_footprint"
    _LOCAL_ODOM_PARENT_FRAME = "base_footprint"
    _LOCAL_ODOM_CHILD_FRAME = "base_link"

    def __init__(
        self,
        *,
        cache_seconds,
    ):
        self._closed = False
        self._validation_lock = threading.Lock()
        self._validation_error = None

        self.node = Node(
            "transient_navigation_preflight_"
            + uuid.uuid4().hex[:10]
        )
        self.buffer = Buffer(
            cache_time=Duration(
                seconds=float(cache_seconds)
            ),
            node=self.node,
        )

        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_subscription = self.node.create_subscription(
            Odometry,
            "/odom",
            self._handle_odom,
            10,
        )
        self.local_odom_subscription = (
            self.node.create_subscription(
                Odometry,
                "/odom/local",
                self._handle_local_odom,
                10,
            )
        )
        self.static_tf_subscription = (
            self.node.create_subscription(
                TFMessage,
                "/tf_static",
                self._handle_static_tf,
                static_qos,
            )
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self.executor.spin,
            name=(
                self.node.get_name()
                + "_executor"
            ),
            daemon=True,
        )
        self.thread.start()

    def _set_validation_error(self, message):
        with self._validation_lock:
            if self._validation_error is None:
                self._validation_error = message

    def validation_error(self):
        with self._validation_lock:
            return self._validation_error

    def _insert_odometry_transform(
        self,
        message,
        *,
        parent_frame,
        child_frame,
    ):
        if (
            message.header.frame_id != parent_frame
            or message.child_frame_id != child_frame
        ):
            self._set_validation_error(
                "Unexpected odometry frame relationship: "
                f"expected {parent_frame} -> {child_frame}, got "
                f"{message.header.frame_id} -> "
                f"{message.child_frame_id}."
            )
            return

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = (
            message.pose.pose.position.x
        )
        transform.transform.translation.y = (
            message.pose.pose.position.y
        )
        transform.transform.translation.z = (
            message.pose.pose.position.z
        )
        transform.transform.rotation.x = (
            message.pose.pose.orientation.x
        )
        transform.transform.rotation.y = (
            message.pose.pose.orientation.y
        )
        transform.transform.rotation.z = (
            message.pose.pose.orientation.z
        )
        transform.transform.rotation.w = (
            message.pose.pose.orientation.w
        )

        self.buffer.set_transform(
            transform,
            "transient_navigation_preflight",
        )

    def _handle_odom(self, message):
        self._insert_odometry_transform(
            message,
            parent_frame=self._ODOM_PARENT_FRAME,
            child_frame=self._ODOM_CHILD_FRAME,
        )

    def _handle_local_odom(self, message):
        self._insert_odometry_transform(
            message,
            parent_frame=self._LOCAL_ODOM_PARENT_FRAME,
            child_frame=self._LOCAL_ODOM_CHILD_FRAME,
        )

    def _handle_static_tf(self, message):
        for transform in message.transforms:
            self.buffer.set_transform_static(
                transform,
                "transient_navigation_preflight",
            )

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
        *,
        timeout,
    ):
        return self.buffer.lookup_transform(
            target_frame,
            source_frame,
            time,
            timeout=timeout,
        )

    def close(self):
        if self._closed:
            return

        self._closed = True
        executor = self.executor
        thread = self.thread
        node = self.node
        subscriptions = (
            self.odom_subscription,
            self.local_odom_subscription,
            self.static_tf_subscription,
        )

        self.executor = None
        self.thread = None
        self.node = None
        self.odom_subscription = None
        self.local_odom_subscription = None
        self.static_tf_subscription = None

        if executor is not None:
            executor.shutdown(timeout_sec=1.0)

        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        if node is not None:
            for subscription in subscriptions:
                if subscription is not None:
                    node.destroy_subscription(subscription)

            if executor is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass

            node.destroy_node()


class TransientOdometryTfLookup:
    """Demand-driven odometry/static-TF lookup for navigation preflight."""

    def __init__(
        self,
        *,
        cache_seconds=2.0,
    ):
        cache_seconds = float(cache_seconds)

        if cache_seconds <= 0.0:
            raise ValueError("cache_seconds must be positive")

        self.cache_seconds = cache_seconds

    @contextmanager
    def session(self):
        session = _TransientOdometryTfSession(
            cache_seconds=self.cache_seconds,
        )

        try:
            yield session
        finally:
            session.close()
