#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import math
import threading

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


class PlanningPathError(RuntimeError):
    """A path could not be computed safely."""


class PlanningPathValidationError(PlanningPathError):
    """A compute-path request was invalid."""


class PlanningPathConflictError(PlanningPathError):
    """A compute-path request conflicts with runtime state."""


class PlanningPathUnavailableError(PlanningPathError):
    """The Nav2 planning action is unavailable."""


class PlanningPathTimeoutError(PlanningPathError):
    """The bounded planning request timed out."""


class PlanningPathService:
    """Compute read-only Nav2 paths without executing them."""

    ACTION_NAME = '/compute_path_to_pose'
    FRAME_ID = 'map'
    PLANNER_ID = 'GridBased'
    SERVER_TIMEOUT_SECONDS = 4.0
    GOAL_TIMEOUT_SECONDS = 4.0
    RESULT_TIMEOUT_SECONDS = 12.0

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
            ComputePathToPose,
            self.ACTION_NAME,
        )
        self._request_lock = threading.Lock()

    @staticmethod
    def _finite_number(value, name):
        if isinstance(value, bool):
            raise PlanningPathValidationError(
                f'{name} must be a finite number.'
            )

        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise PlanningPathValidationError(
                f'{name} must be a finite number.'
            ) from exc

        if not math.isfinite(number):
            raise PlanningPathValidationError(
                f'{name} must be a finite number.'
            )

        return number

    @classmethod
    def validate_request(cls, payload):
        if not isinstance(payload, dict):
            raise PlanningPathValidationError(
                'A JSON object is required.'
            )

        allowed = {
            'goal_x',
            'goal_y',
            'goal_yaw',
        }
        unexpected = sorted(
            set(payload) - allowed
        )

        if unexpected:
            raise PlanningPathValidationError(
                'Unsupported request fields: '
                + ', '.join(unexpected)
            )

        if 'goal_x' not in payload:
            raise PlanningPathValidationError(
                'goal_x is required.'
            )

        if 'goal_y' not in payload:
            raise PlanningPathValidationError(
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

    @staticmethod
    def _wait_future(future, timeout_seconds):
        complete = threading.Event()
        outcome = {}

        def finished(done_future):
            try:
                outcome['result'] = (
                    done_future.result()
                )
            except Exception as exc:
                outcome['error'] = exc
            finally:
                complete.set()

        future.add_done_callback(finished)

        if not complete.wait(timeout_seconds):
            raise PlanningPathTimeoutError(
                'Planning request timed out.'
            )

        if 'error' in outcome:
            raise PlanningPathError(
                f'Planning action failed: '
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

    @staticmethod
    def _yaw_from_quaternion(orientation):
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    @classmethod
    def _serialize_path(
        cls,
        path,
        planning_time,
        request_values,
    ):
        points = []
        length = 0.0
        previous = None

        for index, pose_stamped in enumerate(
            path.poses
        ):
            position = pose_stamped.pose.position
            orientation = (
                pose_stamped.pose.orientation
            )

            point = {
                'index': index,
                'x': float(position.x),
                'y': float(position.y),
                'yaw': float(
                    cls._yaw_from_quaternion(
                        orientation
                    )
                ),
            }

            if previous is not None:
                length += math.hypot(
                    point['x'] - previous['x'],
                    point['y'] - previous['y'],
                )

            points.append(point)
            previous = point

        planning_seconds = (
            float(planning_time.sec)
            + float(planning_time.nanosec)
            / 1_000_000_000.0
        )

        start = (
            dict(points[0])
            if points
            else None
        )
        goal = (
            dict(points[-1])
            if points
            else None
        )

        return {
            'status': 'PATH_READY',
            'read_only': True,
            'executed': False,
            'motion_enabled': False,
            'frame_id': cls.FRAME_ID,
            'planner_id': cls.PLANNER_ID,
            'requested_goal': {
                'x': request_values['goal_x'],
                'y': request_values['goal_y'],
                'yaw': request_values[
                    'goal_yaw'
                ],
            },
            'start': start,
            'goal': goal,
            'pose_count': len(points),
            'length_meters': float(length),
            'planning_time_seconds': (
                planning_seconds
            ),
            'poses': points,
        }

    def compute(self, payload):
        values = self.validate_request(payload)

        if not self._request_lock.acquire(
            blocking=False
        ):
            raise PlanningPathConflictError(
                'Another path request is already active.'
            )

        try:
            if not self._client.wait_for_server(
                timeout_sec=(
                    self.SERVER_TIMEOUT_SECONDS
                )
            ):
                raise PlanningPathUnavailableError(
                    'Compute-path action server is '
                    'unavailable.'
                )

            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = (
                self.FRAME_ID
            )
            clock_stamp = (
                self._node
                .get_clock()
                .now()
                .to_msg()
            )
            goal.goal.header.stamp.sec = int(
                clock_stamp.sec
            )
            goal.goal.header.stamp.nanosec = int(
                clock_stamp.nanosec
            )
            goal.goal.pose.position.x = (
                values['goal_x']
            )
            goal.goal.pose.position.y = (
                values['goal_y']
            )
            goal.goal.pose.position.z = 0.0

            quaternion = (
                self._quaternion_from_yaw(
                    values['goal_yaw']
                )
            )
            goal.goal.pose.orientation.x = 0.0
            goal.goal.pose.orientation.y = 0.0
            goal.goal.pose.orientation.z = (
                quaternion['z']
            )
            goal.goal.pose.orientation.w = (
                quaternion['w']
            )

            goal.planner_id = self.PLANNER_ID
            goal.use_start = False

            goal_handle = self._wait_future(
                self._client.send_goal_async(goal),
                self.GOAL_TIMEOUT_SECONDS,
            )

            if (
                goal_handle is None
                or not goal_handle.accepted
            ):
                raise PlanningPathError(
                    'Compute-path request was rejected.'
                )

            try:
                wrapped_result = self._wait_future(
                    goal_handle.get_result_async(),
                    self.RESULT_TIMEOUT_SECONDS,
                )
            except PlanningPathTimeoutError:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                raise

            if wrapped_result is None:
                raise PlanningPathError(
                    'Compute-path returned no result.'
                )

            if (
                wrapped_result.status
                != GoalStatus.STATUS_SUCCEEDED
            ):
                raise PlanningPathError(
                    'Compute-path did not succeed; '
                    f'action status='
                    f'{wrapped_result.status}.'
                )

            result = wrapped_result.result

            if result is None:
                raise PlanningPathError(
                    'Compute-path returned an empty '
                    'result.'
                )

            if not result.path.poses:
                raise PlanningPathError(
                    'Planner returned no usable path.'
                )

            return self._serialize_path(
                path=result.path,
                planning_time=(
                    result.planning_time
                ),
                request_values=values,
            )

        finally:
            self._request_lock.release()
