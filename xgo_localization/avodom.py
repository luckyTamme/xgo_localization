"""Applied-velocity odometry — pure math, no ROS imports.

Dead-reckons the robot pose from a *commanded, clamped* body velocity plus a
fused IMU heading. The driver publishes the velocity it actually applied after
clamping, which is already metric, so no calibration is needed.

Method: hold the latest velocity (it is sparse — published only when a new
command arrives) and, on every IMU tick, integrate the held body velocity along
the current heading::

    x += (vx * cos(yaw) - vy * sin(yaw)) * dt
    y += (vx * sin(yaw) + vy * cos(yaw)) * dt

Zero commanded velocity means no integration, so the pose is pinned exactly
while the robot stands still. That standstill lock is load-bearing: it is what
keeps a scan-matching SLAM backend from manufacturing phantom motion at rest.
Do not "improve" it into a filtered or smoothed velocity.

This module is deliberately free of ``rclpy`` so the integration maths can be
exercised without a ROS installation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ['Pose2D', 'AppliedVelocityOdometry', 'yaw_from_quaternion']


def wrap_angle(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z) of a quaternion, in radians.

    The quaternion is normalised first: the closed form below assumes unit norm
    and returns a quietly wrong angle otherwise.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the odom frame."""

    x: float
    y: float
    yaw: float


class AppliedVelocityOdometry:
    """Integrate a held body velocity along a heading.

    :param max_dt_s: integration steps longer than this are skipped. Guards
        against pauses, dropped messages and replay hiccups producing a single
        enormous jump.
    :param use_imu_heading: ``True`` takes yaw from the IMU's fused orientation
        (the accurate path). ``False`` dead-reckons yaw by integrating the
        commanded yaw rate instead, which is useful as an A/B and as a fallback
        when no usable orientation is available.
    :param scale_vx: multiplier on forward velocity. The applied velocity is
        already metric, so this exists only for platforms where it is not.
    :param scale_vy: multiplier on lateral velocity.
    :param loop_back_s: a timestamp jumping backwards by more than this many
        seconds is treated as a replay loop and resets the pose to the origin.
    """

    def __init__(
        self,
        max_dt_s: float = 0.5,
        use_imu_heading: bool = True,
        scale_vx: float = 1.0,
        scale_vy: float = 1.0,
        loop_back_s: float = 1.0,
    ) -> None:
        self.max_dt_s = max_dt_s
        self.use_imu_heading = use_imu_heading
        self.scale_vx = scale_vx
        self.scale_vy = scale_vy
        self.loop_back_s = loop_back_s

        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_t: float | None = None
        # The IMU reports an absolute fused heading whose zero is wherever the
        # firmware happened to start. Subtracting the first reading makes the
        # odom frame start at identity, so "yaw" means "turned since startup".
        self._yaw_offset: float | None = None

    # ------------------------------------------------------------------ state

    @property
    def velocity(self) -> tuple[float, float, float]:
        """The currently held ``(vx, vy, vyaw)`` in the body frame."""
        return self._vx, self._vy, self._vyaw

    @property
    def pose(self) -> Pose2D:
        """The integrated pose. ``yaw`` is only meaningful after an update."""
        return Pose2D(self._x, self._y, self._yaw)

    def reset(self) -> None:
        """Return the pose to the origin and forget the timestamp history.

        The heading offset is dropped too, so the next reading re-zeroes the
        frame — which is what a replay starting over should do.
        """
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._yaw_offset = None
        self._prev_t = None

    # ------------------------------------------------------------------ inputs

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """Replace the held body velocity.

        Called whenever a new applied-velocity sample arrives. Between calls the
        last value is held (zero-order hold), because the source publishes only
        on change, not at a fixed rate.
        """
        self._vx = vx * self.scale_vx
        self._vy = vy * self.scale_vy
        self._vyaw = vyaw

    def update(self, t: float, quaternion: tuple[float, float, float, float] | None = None) -> Pose2D | None:
        """Advance the integration to time ``t`` and return the new pose.

        :param t: timestamp of the tick, in seconds.
        :param quaternion: ``(x, y, z, w)`` orientation. Required when
            ``use_imu_heading`` is set; ignored otherwise.
        :returns: the updated pose, or ``None`` when the tick was rejected
            because a heading was required but the quaternion was unusable.

        A tick with no valid ``dt`` (the first one, one that is too long, or one
        that goes backwards) still produces a pose — it simply does not
        integrate. That keeps the output continuous across gaps instead of
        stalling.
        """
        heading_ok = quaternion is not None and _is_usable_quaternion(quaternion)
        if self.use_imu_heading and not heading_ok:
            return None

        dt = self._advance_clock(t)

        if self.use_imu_heading:
            assert quaternion is not None  # guaranteed by heading_ok
            absolute_yaw = yaw_from_quaternion(*quaternion)
            if self._yaw_offset is None:
                self._yaw_offset = absolute_yaw
            yaw = wrap_angle(absolute_yaw - self._yaw_offset)
        else:
            yaw = self._yaw
            if dt is not None:
                yaw = wrap_angle(yaw + self._vyaw * dt)

        if dt is not None:
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            self._x += (self._vx * cos_yaw - self._vy * sin_yaw) * dt
            self._y += (self._vx * sin_yaw + self._vy * cos_yaw) * dt

        self._yaw = yaw
        return Pose2D(self._x, self._y, yaw)

    # ----------------------------------------------------------------- helpers

    def _advance_clock(self, t: float) -> float | None:
        """Return the usable ``dt`` for this tick, or ``None`` to skip integration."""
        previous, self._prev_t = self._prev_t, t
        if previous is None:
            return None

        delta = t - previous
        if delta < -self.loop_back_s:
            # The clock jumped backwards a long way: a bag replay looped. Start
            # the trajectory over rather than integrating a nonsense interval.
            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._yaw_offset = None
            return None
        if 0.0 < delta <= self.max_dt_s:
            return delta
        return None


def _is_usable_quaternion(q: tuple[float, float, float, float]) -> bool:
    """Reject the all-zero and non-finite orientations the firmware can emit."""
    norm_squared = sum(component * component for component in q)
    return math.isfinite(norm_squared) and norm_squared >= 1e-6
