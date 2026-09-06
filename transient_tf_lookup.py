#!/usr/bin/env python3

from contextlib import contextmanager
import threading
import uuid

from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from tf2_ros import Buffer
from tf2_ros import TransformListener


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
