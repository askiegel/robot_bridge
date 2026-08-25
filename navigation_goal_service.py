#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import math
import threading

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationGoalError(RuntimeError):
    """A guarded navigation goal could not complete safely."""


class NavigationGoalValidationError(NavigationGoalError):
    """A guarded navigation request was invalid."""


class NavigationGoalConflictError(NavigationGoalError):
    """Another guarded navigation request is active."""


class NavigationGoalUnavailableError(NavigationGoalError):
    """The guarded navigation action is unavailable."""


class NavigationGoalTimeoutError(NavigationGoalError):
    """The guarded navigation request exceeded its limit."""


class NavigationGoalCancelledError(NavigationGoalError):
    """The guarded navigation request was cancelled."""


class NavigationGoalService:
    """Execute one fixed, short, recovery-free Nav2 goal."""

    ACTION_NAME = '/navigate_to_pose'
    FRAME_ID = 'map'
    MAXIMUM_GOAL_DISTANCE_METERS = 0.50
    MAXIMUM_EXECUTION_SECONDS = 25.0
    MAXIMUM_POSE_AGE_SECONDS = 3.0
    SERVER_TIMEOUT_SECONDS = 4.0
    GOAL_TIMEOUT_SECONDS = 4.0
    CANCEL_TIMEOUT_SECONDS = 4.0
    BEHAVIOR_TREE = (
        '/home/ubuntu/ros2_ws/install/'
        'mini_pupper_navigation/share/'
        'mini_pupper_navigation/behavior_trees/'
        'mayday_guarded_navigate_to_pose.xml'
    )

    def __init__(
        self,
        node,
        action_client_factory=None,
    ):
        self._node = node
        self._action_client_factory = (
            action_client_factory or ActionClient
        )
        self._client = self._action_client_factory(
            node,
            NavigateToPose,
            self.ACTION_NAME,
        )
        self._request_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._active_goal_handle = None
        self._cancel_requested = False

    def action_server_ready(self):
        """Return whether the existing NavigateToPose server is ready."""
        return bool(
            self._client.wait_for_server(
                timeout_sec=0.0
            )
        )

    @staticmethod
    def _finite_number(value, name):
        if isinstance(value, bool):
            raise NavigationGoalValidationError(
                f'{name} must be a finite number.'
            )

        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise NavigationGoalValidationError(
                f'{name} must be a finite number.'
            ) from exc

        if not math.isfinite(number):
            raise NavigationGoalValidationError(
                f'{name} must be a finite number.'
            )

        return number

    @classmethod
    def validate_request(cls, payload):
        if not isinstance(payload, dict):
            raise NavigationGoalValidationError(
                'A JSON object is required.'
            )

        allowed = {
            'goal_x',
            'goal_y',
            'goal_yaw',
        }
        unexpected = sorted(set(payload) - allowed)

        if unexpected:
            raise NavigationGoalValidationError(
                'Unsupported request fields: '
                + ', '.join(unexpected)
            )

        if 'goal_x' not in payload:
            raise NavigationGoalValidationError(
                'goal_x is required.'
            )

        if 'goal_y' not in payload:
            raise NavigationGoalValidationError(
                'goal_y is required.'
            )

        return {
            'goal_x': cls._finite_number(
                payload['goal_x'],
                'goal_x',
            ),
            'goal_y': cls._finite_number(
                payload['goal_y'],
                'goal_y',
            ),
            'goal_yaw': cls._finite_number(
                payload.get('goal_yaw', 0.0),
                'goal_yaw',
            ),
        }

    @classmethod
    def validate_pose(cls, snapshot):
        if not isinstance(snapshot, dict):
            raise NavigationGoalValidationError(
                'Fresh localization is required.'
            )

        if snapshot.get('available') is not True:
            raise NavigationGoalValidationError(
                'Fresh localization is required.'
            )

        age = cls._finite_number(
            snapshot.get('age_seconds'),
            'localization age',
        )

        if age > cls.MAXIMUM_POSE_AGE_SECONDS:
            raise NavigationGoalValidationError(
                'Localization pose is stale.'
            )

        pose = snapshot.get('pose')

        if not isinstance(pose, dict):
            raise NavigationGoalValidationError(
                'A map-frame localization pose is required.'
            )

        if pose.get('frame_id') != cls.FRAME_ID:
            raise NavigationGoalValidationError(
                'Localization pose must use the map frame.'
            )

        position = pose.get('position')

        if not isinstance(position, dict):
            raise NavigationGoalValidationError(
                'Localization position is unavailable.'
            )

        return {
            'x': cls._finite_number(
                position.get('x'),
                'localization x',
            ),
            'y': cls._finite_number(
                position.get('y'),
                'localization y',
            ),
            'age_seconds': age,
        }

    @classmethod
    def validate_bounded_goal(
        cls,
        request_values,
        pose_values,
    ):
        distance = math.hypot(
            request_values['goal_x'] - pose_values['x'],
            request_values['goal_y'] - pose_values['y'],
        )

        if distance > cls.MAXIMUM_GOAL_DISTANCE_METERS:
            raise NavigationGoalValidationError(
                'Goal exceeds the fixed 0.50-meter limit.'
            )

        return float(distance)

    @staticmethod
    def _wait_future(
        future,
        timeout_seconds,
        timeout_message,
    ):
        complete = threading.Event()
        outcome = {}

        def finished(done_future):
            try:
                outcome['result'] = done_future.result()
            except Exception as exc:
                outcome['error'] = exc
            finally:
                complete.set()

        future.add_done_callback(finished)

        if not complete.wait(timeout_seconds):
            raise NavigationGoalTimeoutError(
                timeout_message
            )

        if 'error' in outcome:
            raise NavigationGoalError(
                f'Navigation action failed: '
                f'{outcome["error"]}'
            ) from outcome['error']

        return outcome.get('result')

    @staticmethod
    def _quaternion_from_yaw(yaw):
        half = float(yaw) / 2.0

        return {
            'z': math.sin(half),
            'w': math.cos(half),
        }

    def _cancel_handle(self, goal_handle):
        if goal_handle is None:
            return False

        try:
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_future(
                cancel_future,
                self.CANCEL_TIMEOUT_SECONDS,
                'Navigation cancellation timed out.',
            )
        except Exception:
            return False

        return True

    def cancel_active(self):
        with self._state_lock:
            self._cancel_requested = True
            goal_handle = self._active_goal_handle
            active = self._request_lock.locked()

        cancelled = self._cancel_handle(goal_handle)

        return {
            'active': bool(active),
            'cancel_requested': bool(active),
            'cancel_signal_sent': bool(cancelled),
        }

    def execute(self, payload, pose_snapshot):
        values = self.validate_request(payload)
        pose_values = self.validate_pose(pose_snapshot)
        requested_distance = self.validate_bounded_goal(
            values,
            pose_values,
        )

        if not self._request_lock.acquire(blocking=False):
            raise NavigationGoalConflictError(
                'Another navigation goal is already active.'
            )

        with self._state_lock:
            self._cancel_requested = False
            self._active_goal_handle = None

        try:
            if not self._client.wait_for_server(
                timeout_sec=self.SERVER_TIMEOUT_SECONDS
            ):
                raise NavigationGoalUnavailableError(
                    'Navigate-to-pose action server is unavailable.'
                )

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.FRAME_ID

            clock_stamp = (
                self._node
                .get_clock()
                .now()
                .to_msg()
            )
            goal.pose.header.stamp.sec = int(clock_stamp.sec)
            goal.pose.header.stamp.nanosec = int(
                clock_stamp.nanosec
            )

            goal.pose.pose.position.x = values['goal_x']
            goal.pose.pose.position.y = values['goal_y']
            goal.pose.pose.position.z = 0.0

            quaternion = self._quaternion_from_yaw(
                values['goal_yaw']
            )
            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = quaternion['z']
            goal.pose.pose.orientation.w = quaternion['w']
            goal.behavior_tree = self.BEHAVIOR_TREE

            goal_handle = self._wait_future(
                self._client.send_goal_async(goal),
                self.GOAL_TIMEOUT_SECONDS,
                'Navigation goal submission timed out.',
            )

            if goal_handle is None or not goal_handle.accepted:
                raise NavigationGoalError(
                    'Navigation goal was rejected.'
                )

            with self._state_lock:
                self._active_goal_handle = goal_handle
                cancel_requested = self._cancel_requested

            if cancel_requested:
                self._cancel_handle(goal_handle)
                raise NavigationGoalCancelledError(
                    'Navigation goal was cancelled.'
                )

            try:
                wrapped_result = self._wait_future(
                    goal_handle.get_result_async(),
                    self.MAXIMUM_EXECUTION_SECONDS,
                    'Navigation exceeded the fixed '
                    '15-second execution limit.',
                )
            except NavigationGoalTimeoutError:
                self._cancel_handle(goal_handle)
                raise

            if wrapped_result is None:
                raise NavigationGoalError(
                    'Navigation returned no result.'
                )

            if wrapped_result.status == GoalStatus.STATUS_CANCELED:
                raise NavigationGoalCancelledError(
                    'Navigation goal was cancelled.'
                )

            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                raise NavigationGoalError(
                    'Navigation did not succeed; '
                    f'action status={wrapped_result.status}.'
                )

            return {
                'status': 'NAVIGATION_SUCCEEDED',
                'executed': True,
                'bounded': True,
                'retries_requested': 0,
                'recoveries_requested': 0,
                'frame_id': self.FRAME_ID,
                'behavior_tree': self.BEHAVIOR_TREE,
                'maximum_goal_distance_meters': (
                    self.MAXIMUM_GOAL_DISTANCE_METERS
                ),
                'maximum_execution_seconds': (
                    self.MAXIMUM_EXECUTION_SECONDS
                ),
                'pose_age_seconds': pose_values[
                    'age_seconds'
                ],
                'start': {
                    'x': pose_values['x'],
                    'y': pose_values['y'],
                },
                'goal': {
                    'x': values['goal_x'],
                    'y': values['goal_y'],
                    'yaw': values['goal_yaw'],
                },
                'requested_distance_meters': (
                    requested_distance
                ),
            }

        finally:
            with self._state_lock:
                self._active_goal_handle = None
                self._cancel_requested = False

            self._request_lock.release()
