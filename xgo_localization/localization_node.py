#!/usr/bin/env python3
"""Localisation for the XGO2: where the robot is relative to where it started.

Runs applied-velocity odometry and a pose arbiter in one process. A SLAM backend
(cartographer or rtabmap, started by the launch file) corrects ``map -> odom``
alongside; this node owns everything else.

Frame contract, REP-105::

    map --(SLAM backend)--> odom --(this node)--> base_link --(static)--> base_laser

The odometry layer never gaps, so if the backend stalls or has not started yet
the pose keeps flowing, dead-reckoned, flagged ``DEGRADED`` on ``/diagnostics``.

The pose is composed in-process — the backend's ``map -> odom`` combined with our
own odometry, rather than a TF lookup of ``map -> base_link``. That avoids
lookup jitter and keeps the odometry half exact.
"""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (PoseWithCovarianceStamped, TransformStamped,
                               TwistStamped)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Imu
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformException, TransformListener)

from xgo_localization.arbiter import BackendState, PoseArbiter
from xgo_localization.avodom import (AppliedVelocityOdometry, Pose2D,
                                     wrap_angle, yaw_from_quaternion)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """``(x, y, z, w)`` for a rotation of ``yaw`` about +z."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """``(x, y, z, w)`` from roll-pitch-yaw, applied Z-Y-X."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _pose_from_transform(tf: TransformStamped) -> Pose2D:
    """Planar pose of a TransformStamped."""
    translation = tf.transform.translation
    rotation = tf.transform.rotation
    return Pose2D(
        translation.x,
        translation.y,
        yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def compose_2d(outer: Pose2D, inner: Pose2D) -> Pose2D:
    """Chain two planar transforms: ``outer`` then ``inner``."""
    cos_yaw, sin_yaw = math.cos(outer.yaw), math.sin(outer.yaw)
    return Pose2D(
        x=outer.x + inner.x * cos_yaw - inner.y * sin_yaw,
        y=outer.y + inner.x * sin_yaw + inner.y * cos_yaw,
        yaw=wrap_angle(outer.yaw + inner.yaw),
    )


class LocalizationNode(Node):
    """Odometry, pose arbitration, and the static sensor transform."""

    def __init__(self) -> None:
        super().__init__('localization_node')

        # -- inputs. Absolute defaults so they resolve at the root namespace
        #    even though this node is pushed into /localization.
        velocity_topic = self._param('velocity_topic', '/xgo/applied_vel')
        self._imu_topic = imu_topic = self._param('imu_topic', '/imu/data')

        # -- frames
        self._map_frame = self._param('map_frame', 'map')
        self._odom_frame = self._param('odom_frame', 'odom')
        self._base_frame = self._param('base_frame', 'base_link')
        self._laser_frame = self._param('laser_frame', 'base_laser')

        # -- laser mount. The LiDAR is a retrofit, so no upstream robot
        #    description contains it and nothing else publishes this transform.
        #    Without it a SLAM backend silently drops every scan.
        laser_transform = self._param(
            'laser_transform', [0.0, 0.0, 0.18, 0.0, 0.0, 0.0])

        # -- odometry
        self._odometry = AppliedVelocityOdometry(
            max_dt_s=self._param('avodom.max_dt_s', 0.5),
            use_imu_heading=self._param('avodom.use_imu_heading', True),
            scale_vx=self._param('avodom.scale_vx', 1.0),
            scale_vy=self._param('avodom.scale_vy', 1.0),
        )

        # -- arbitration
        self._arbiter = PoseArbiter(
            stale_after_s=self._param('arbiter.stale_after_s', 1.0),
            healthy_xy_var=self._param('arbiter.healthy_xy_var', 0.01),
            healthy_yaw_var=self._param('arbiter.healthy_yaw_var', 0.02),
            degraded_xy_var=self._param('arbiter.degraded_xy_var', 1.0),
            degraded_yaw_var=self._param('arbiter.degraded_yaw_var', 0.5),
        )
        publish_rate_hz = self._param('arbiter.publish_rate_hz', 20.0)
        if publish_rate_hz <= 0.0:
            raise ValueError(f'arbiter.publish_rate_hz must be > 0, got {publish_rate_hz}')
        # An input silent for this long is reported as an error: the pose is
        # then frozen, not merely uncorrected.
        self._input_timeout_s = self._param('input_timeout_s', 2.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._laser_transform = laser_transform
        self._publish_laser_transform(laser_transform)

        # Re-send the static transform periodically. In principle the latched
        # /tf_static publication reaches late subscribers by itself, but that
        # relies on every hop preserving transient-local durability. It takes
        # only one intermediary negotiating down to volatile — a bridge facing a
        # second, volatile publisher of /tf_static, say — for a consumer that
        # connects mid-run to never learn where the LiDAR is, silently and
        # permanently. A handful of bytes every few seconds buys that back.
        # Set to 0 to publish once only.
        static_tf_period_s = self._param('static_tf_period_s', 5.0)
        if static_tf_period_s > 0.0:
            self.create_timer(
                static_tf_period_s,
                lambda: self._publish_laser_transform(self._laser_transform, quiet=True))

        # Relative names: the launch file pushes this node into /localization,
        # so these become /localization/odom and /localization/pose.
        self._odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'pose', 20)
        # /diagnostics is a shared bus by convention — tooling subscribes to it
        # absolutely, so it must not be namespaced.
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # The applied velocity is published RELIABLE and TRANSIENT_LOCAL. We
        # subscribe RELIABLE + VOLATILE on purpose: reliable matches the
        # publisher, and volatile is compatible with either durability while
        # deliberately skipping the latched pre-run sample, so we start from
        # rest rather than from a command left over from before startup.
        velocity_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(TwistStamped, velocity_topic, self._on_velocity, velocity_qos)
        # Sensor-data QoS (BEST_EFFORT) for the IMU. A best-effort subscription
        # is compatible with both best-effort and reliable publishers, whereas a
        # reliable one silently receives nothing from a best-effort driver — and
        # a silent IMU means no odometry and no TF at all.
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / publish_rate_hz, self._publish_pose)

        self._last_state: BackendState | None = None
        self._last_imu_t: float | None = None
        self._last_velocity_t: float | None = None
        self._last_correction: Pose2D | None = None
        self.get_logger().info(
            f'localisation up: {velocity_topic} + {imu_topic} -> '
            f'{self._map_frame} pose at {publish_rate_hz:g} Hz')

    # --------------------------------------------------------------- parameters

    def _param(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ startup

    def _publish_laser_transform(self, xyz_rpy, quiet: bool = False) -> None:
        if len(xyz_rpy) != 6:
            raise ValueError(
                f'laser_transform needs 6 values (x y z roll pitch yaw), got {len(xyz_rpy)}')
        x, y, z, roll, pitch, yaw = (float(v) for v in xyz_rpy)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._laser_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._static_tf_broadcaster.sendTransform(transform)

        if quiet:
            return
        self.get_logger().info(
            f'publishing static {self._base_frame} -> {self._laser_frame}: '
            f'xyz=({x:g}, {y:g}, {z:g}) rpy=({roll:g}, {pitch:g}, {yaw:g}). '
            'Measure this per robot — a tilted mount biases every range.')

    # ------------------------------------------------------------------- inputs

    def _on_velocity(self, msg: TwistStamped) -> None:
        self._last_velocity_t = self._now_s()
        self._odometry.set_velocity(
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _on_imu(self, msg: Imu) -> None:
        """Integrate one step and publish odometry.

        The IMU is the integration tick as well as the heading source: it is the
        only sensor on this platform that arrives at a steady rate. Note that the
        firmware populates ``orientation`` only — ``angular_velocity`` and
        ``linear_acceleration`` are zero and flagged invalid, so we never read them.
        """
        self._last_imu_t = self._now_s()
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        orientation = msg.orientation
        pose = self._odometry.update(
            t, (orientation.x, orientation.y, orientation.z, orientation.w))
        if pose is None:
            # No usable orientation and the heading comes from the IMU, so this
            # tick cannot be integrated. Throttled: if the firmware wedges, this
            # fires every tick and would otherwise drown the log.
            self.get_logger().warn(
                'IMU orientation unusable; skipping integration',
                throttle_duration_sec=5.0)
            return

        vx, vy, vyaw = self._odometry.velocity
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Twist is expressed in the child (body) frame, per REP-105.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vyaw
        # Nominal per-step odometry uncertainty. Twist covariance is left at
        # zero ("unknown") rather than reusing the position variances, which
        # would be a different quantity wearing the same numbers.
        odom.pose.covariance = self._arbiter.covariance(BackendState.HEALTHY)
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------------------ outputs

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_map_to_odom(self) -> Pose2D:
        """The backend's correction, held at its last value if the lookup fails.

        Holding rather than snapping to identity is the point of the whole
        design: when the backend stops, the correction freezes and odometry
        carries on underneath, so the pose stays continuous instead of jumping
        back to the uncorrected estimate for one tick.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._odom_frame, rclpy.time.Time())
        except TransformException:
            # Identity only until the backend has ever spoken; after that the
            # last correction is the best available answer.
            return self._last_correction or Pose2D(0.0, 0.0, 0.0)

        stamp = tf.header.stamp
        self._arbiter.note_backend_correction(stamp.sec + stamp.nanosec * 1e-9)
        self._last_correction = _pose_from_transform(tf)
        return self._last_correction

    def _lookup_map_to_base(self) -> Pose2D | None:
        """The global pose straight from tf2, or ``None`` if the chain is broken.

        Preferred over composing ``map->odom`` with our own odometry by hand:
        tf2 evaluates both halves of the chain at a consistent time, whereas
        pairing the newest correction with the newest odometry silently mixes
        two different instants. That mismatch is invisible when the backend
        corrects quickly and badly wrong when it does not.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return _pose_from_transform(tf)

    def _publish_pose(self) -> None:
        """Emit the global pose and health, at a constant rate regardless of the backend."""
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        # Track backend freshness first: this is what decides health, and it
        # refreshes the cached correction used by the fallback below.
        correction = self._lookup_map_to_odom()

        # Prefer tf2's own composition — it evaluates map->odom and
        # odom->base_link at a consistent time. Fall back to composing by hand
        # only when the chain is incomplete, which is the case before the
        # backend has converged (correction is then identity, so the pose is
        # simply uncorrected odometry — still relative to where we started).
        pose = self._lookup_map_to_base()
        if pose is None:
            pose = compose_2d(correction, self._odometry.pose)
        state, reason = self._arbiter.describe(now_s)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = self._arbiter.covariance(state)
        self._pose_pub.publish(msg)

        self._publish_diagnostics(now, state, reason, pose, now_s)

        if state is not self._last_state:
            # Two call sites on purpose. rclpy caches a logger's severity per
            # source location, so logging both levels from one line raises
            # "Logger severity cannot be changed between calls" the first time
            # the state flips.
            if state.is_healthy:
                self.get_logger().info(f'localisation {state.value}: {reason}')
            else:
                self.get_logger().warn(f'localisation {state.value}: {reason}')
            self._last_state = state

    def _publish_diagnostics(
        self, now, state: BackendState, reason: str, pose: Pose2D, now_s: float
    ) -> None:
        age = self._arbiter.seconds_since_correction(now_s)
        imu_age = None if self._last_imu_t is None else now_s - self._last_imu_t
        velocity_age = None if self._last_velocity_t is None else now_s - self._last_velocity_t

        # Backend health and input health are different failures. A dead IMU
        # means the pose is frozen, not merely uncorrected, and reporting that
        # as a mere DEGRADED would claim dead reckoning is working when it is not.
        imu_dead = imu_age is None or imu_age > self._input_timeout_s
        if imu_dead:
            level = DiagnosticStatus.ERROR
            message = (f'no IMU on {self._imu_topic} '
                       f'({"never" if imu_age is None else f"{imu_age:.1f} s"}) — '
                       'odometry is not running; pose is frozen')
        elif state.is_healthy:
            level = DiagnosticStatus.OK
            message = reason
        else:
            level = DiagnosticStatus.WARN
            message = reason

        status = DiagnosticStatus()
        # Namespaced inside the message rather than on the topic, so standard
        # diagnostics tooling groups it correctly.
        status.name = 'localization: backend'
        status.hardware_id = 'xgo_localization'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='state', value=state.value),
            KeyValue(key='map_to_odom_age_s', value='never' if age is None else f'{age:.3f}'),
            KeyValue(key='ever_corrected', value=str(self._arbiter.has_ever_corrected).lower()),
            KeyValue(key='imu_age_s', value='never' if imu_age is None else f'{imu_age:.2f}'),
            KeyValue(key='applied_vel_age_s',
                     value='never' if velocity_age is None else f'{velocity_age:.2f}'),
            KeyValue(key='pose_x', value=f'{pose.x:.3f}'),
            KeyValue(key='pose_y', value=f'{pose.y:.3f}'),
            KeyValue(key='pose_yaw_deg', value=f'{math.degrees(pose.yaw):.1f}'),
        ]

        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Construction is inside the try so a bad parameter still shuts rclpy
        # down cleanly instead of leaving a bare traceback.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
