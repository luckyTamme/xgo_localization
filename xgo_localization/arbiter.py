"""Backend health tracking and covariance policy — pure logic, no ROS imports.

The SLAM backend corrects ``map -> odom``. When it stalls, dies, or has not
started yet, that correction goes stale while the odometry underneath keeps
running. This module decides which of those situations we are in and what
uncertainty to advertise, so the node can keep publishing a pose at a constant
rate no matter what the backend is doing.

Covariance is deliberately a two-state constant, not a growth curve. TF carries
no covariance for us to propagate, so any smooth model would be invented
precision. Healthy and degraded are the only distinctions a consumer can act on.
"""

from __future__ import annotations

import enum

__all__ = ['BackendState', 'PoseArbiter']

# Indices into a ROS 6x6 row-major covariance matrix.
_COV_XX = 0
_COV_YY = 7
_COV_YAW = 35


class BackendState(enum.Enum):
    """How much to trust the current pose."""

    #: The backend is publishing corrections; pose is globally referenced.
    HEALTHY = 'healthy'
    #: No fresh correction. Pose is still published, dead-reckoned from odometry.
    DEGRADED = 'degraded'

    @property
    def is_healthy(self) -> bool:
        return self is BackendState.HEALTHY


class PoseArbiter:
    """Track backend freshness and hand out the matching covariance.

    :param stale_after_s: how long without a backend correction before the pose
        is considered degraded.
    :param healthy_xy_var: position variance (m^2) while the backend is healthy.
    :param healthy_yaw_var: yaw variance (rad^2) while the backend is healthy.
    :param degraded_xy_var: position variance (m^2) while dead-reckoning.
    :param degraded_yaw_var: yaw variance (rad^2) while dead-reckoning.
    """

    def __init__(
        self,
        stale_after_s: float = 1.0,
        healthy_xy_var: float = 0.01,
        healthy_yaw_var: float = 0.02,
        degraded_xy_var: float = 1.0,
        degraded_yaw_var: float = 0.5,
        future_tolerance_s: float = 0.5,
    ) -> None:
        self.stale_after_s = stale_after_s
        self.future_tolerance_s = future_tolerance_s
        self.healthy_xy_var = healthy_xy_var
        self.healthy_yaw_var = healthy_yaw_var
        self.degraded_xy_var = degraded_xy_var
        self.degraded_yaw_var = degraded_yaw_var

        self._last_correction_t: float | None = None
        self._ever_corrected = False

    # ------------------------------------------------------------------ inputs

    def note_backend_correction(self, t: float) -> None:
        """Record that a ``map -> odom`` correction was observed at time ``t``."""
        self._last_correction_t = t
        self._ever_corrected = True

    # ----------------------------------------------------------------- queries

    @property
    def has_ever_corrected(self) -> bool:
        """False until the backend produces its first correction."""
        return self._ever_corrected

    def seconds_since_correction(self, now: float) -> float | None:
        """Signed age of the newest correction, or ``None`` if there is none.

        Deliberately signed. A correction stamped in the *future* means the node
        and the backend are reading different clocks — almost always a
        ``use_sim_time`` mismatch. Clamping that to zero would report the
        healthiest possible state at exactly the moment the setup is broken.
        """
        if self._last_correction_t is None:
            return None
        return now - self._last_correction_t

    def state(self, now: float) -> BackendState:
        """Classify the backend at time ``now``."""
        age = self.seconds_since_correction(now)
        if age is None:
            return BackendState.DEGRADED
        if age > self.stale_after_s or age < -self.future_tolerance_s:
            return BackendState.DEGRADED
        return BackendState.HEALTHY

    def covariance(self, state: BackendState) -> list[float]:
        """A 6x6 row-major covariance matrix for the given state."""
        if state.is_healthy:
            xy_var, yaw_var = self.healthy_xy_var, self.healthy_yaw_var
        else:
            xy_var, yaw_var = self.degraded_xy_var, self.degraded_yaw_var

        covariance = [0.0] * 36
        covariance[_COV_XX] = xy_var
        covariance[_COV_YY] = xy_var
        covariance[_COV_YAW] = yaw_var
        return covariance

    def describe(self, now: float) -> tuple[BackendState, str]:
        """Return the state and a one-line human-readable reason."""
        state = self.state(now)
        age = self.seconds_since_correction(now)
        if age is None:
            return state, 'no map->odom correction yet; dead reckoning from odometry'
        if age < -self.future_tolerance_s:
            return state, (
                f'map->odom is stamped {-age:.1f} s in the future — the backend and '
                'this node are on different clocks. Check use_sim_time.')
        if state.is_healthy:
            # A small negative age is normal: rtabmap stamps its correction a
            # little ahead of now.
            if age < 0.0:
                return state, f'map->odom fresh (stamped {-age:.2f} s ahead)'
            return state, f'map->odom fresh ({age:.2f} s old)'
        return state, f'map->odom stale ({age:.1f} s old); dead reckoning from odometry'
