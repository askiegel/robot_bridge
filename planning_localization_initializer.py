#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel

import threading
import time

from std_srvs.srv import Empty


class PlanningLocalizationError(RuntimeError):
    """Planning localization could not be initialized safely."""


class PlanningLocalizationConflictError(
    PlanningLocalizationError
):
    """Another initialization request is already active."""


class PlanningLocalizationUnavailableError(
    PlanningLocalizationError
):
    """A required fixed AMCL service is unavailable."""


class PlanningLocalizationTimeoutError(
    PlanningLocalizationError
):
    """A bounded AMCL service request timed out."""


class PlanningLocalizationInitializer:
    """
    Initialize stationary AMCL without publishing a pose or motion.

    The service names, request types, update count, and timeouts are
    fixed in source. Browser input cannot select a service, topic,
    pose, transform, command, planner, controller, or velocity.
    """

    GLOBAL_SERVICE = (
        '/reinitialize_global_localization'
    )
    NOMOTION_SERVICE = '/request_nomotion_update'

    SERVICE_TIMEOUT_SECONDS = 6.0
    RESPONSE_TIMEOUT_SECONDS = 6.0
    NOMOTION_UPDATE_COUNT = 20
    NOMOTION_UPDATE_INTERVAL_SECONDS = 1.0

    def __init__(
        self,
        node,
        client_factory=None,
        pose_clearer=None,
        sleeper=None,
    ):
        self._node = node
        self._client_factory = (
            client_factory or node.create_client
        )
        self._pose_clearer = (
            pose_clearer or (lambda: None)
        )
        self._sleeper = sleeper or time.sleep

        self._global_client = self._client_factory(
            Empty,
            self.GLOBAL_SERVICE,
        )
        self._nomotion_client = self._client_factory(
            Empty,
            self.NOMOTION_SERVICE,
        )

        self._request_lock = threading.Lock()

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
            raise PlanningLocalizationTimeoutError(
                'AMCL initialization request timed out.'
            )

        if 'error' in outcome:
            raise PlanningLocalizationError(
                'AMCL initialization request failed: '
                f'{outcome["error"]}'
            ) from outcome['error']

        return outcome.get('result')

    @classmethod
    def _wait_for_service(
        cls,
        client,
        service_name,
    ):
        if not client.wait_for_service(
            timeout_sec=cls.SERVICE_TIMEOUT_SECONDS,
        ):
            raise PlanningLocalizationUnavailableError(
                f'Required AMCL service is unavailable: '
                f'{service_name}'
            )

    @classmethod
    def _call_empty(cls, client):
        future = client.call_async(Empty.Request())

        cls._wait_future(
            future,
            cls.RESPONSE_TIMEOUT_SECONDS,
        )

    def refresh_pose(self):
        """
        Request one fixed stationary AMCL pose refresh.

        This method does not redistribute particles, publish an
        initial pose, calculate or execute a path, start a controller,
        publish velocity, or move the robot.
        """
        if not self._request_lock.acquire(blocking=False):
            raise PlanningLocalizationConflictError(
                'Planning localization request is already active.'
            )

        try:
            self._wait_for_service(
                self._nomotion_client,
                self.NOMOTION_SERVICE,
            )

            # Remove the previous cached pose before requesting one
            # fixed no-motion scan update. Any pose subsequently
            # exposed to the dashboard must come from this refresh.
            self._pose_clearer()
            self._call_empty(self._nomotion_client)

            return {
                'action': 'PLANNING_POSE_REFRESHED',
                'global_localization_requested': False,
                'nomotion_updates_requested': 1,
                'stationary_required': True,
                'pose_published': False,
                'initial_pose_supplied': False,
                'path_computed': False,
                'path_executed': False,
                'navigation_goal_executed': False,
                'controller_enabled': False,
                'navigator_enabled': False,
                'motion_enabled': False,
            }
        finally:
            self._request_lock.release()

    def initialize(self):
        """
        Distribute particles and request bounded stationary updates.

        This method has no request payload. It cannot publish an
        initial pose, calculate or execute a path, start a controller,
        publish velocity, or move the robot.
        """
        if not self._request_lock.acquire(blocking=False):
            raise PlanningLocalizationConflictError(
                'Planning localization initialization is '
                'already active.'
            )

        try:
            self._wait_for_service(
                self._global_client,
                self.GLOBAL_SERVICE,
            )
            self._wait_for_service(
                self._nomotion_client,
                self.NOMOTION_SERVICE,
            )

            self._call_empty(self._global_client)

            # AMCL may publish its configured launch-time pose
            # while particles are being redistributed. Remove
            # that pose now. Only the following no-motion scan
            # updates may repopulate localization telemetry.
            self._pose_clearer()

            completed_updates = 0

            # Allow global particles to process distinct
            # stationary scans. The final update is reserved
            # for verification after another cache clear.
            for _ in range(
                self.NOMOTION_UPDATE_COUNT - 1
            ):
                self._call_empty(
                    self._nomotion_client
                )
                completed_updates += 1
                self._sleeper(
                    self.NOMOTION_UPDATE_INTERVAL_SECONDS
                )

            # Discard every pose observed during convergence.
            # Only the final no-motion request may repopulate
            # telemetry returned to the dashboard.
            self._pose_clearer()
            self._call_empty(
                self._nomotion_client
            )
            completed_updates += 1

            return {
                'action': (
                    'PLANNING_LOCALIZATION_INITIALIZED'
                ),
                'global_localization_requested': True,
                'nomotion_updates_requested': (
                    completed_updates
                ),
                'stationary_required': True,
                'pose_published': False,
                'initial_pose_supplied': False,
                'path_computed': False,
                'path_executed': False,
                'navigation_goal_executed': False,
                'controller_enabled': False,
                'navigator_enabled': False,
                'motion_enabled': False,
            }
        finally:
            self._request_lock.release()
