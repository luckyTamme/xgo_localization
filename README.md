# xgo_localization

> Developed as part of the course **Master Project: Distributed Systems** at
> [DOS](https://www.tu.berlin/dos), TU Berlin.

Localisation for the **XGO2 quadruped**: where the robot is relative to where it
started, published continuously, with a selectable SLAM backend.

One node, standard message types, no custom interfaces. Drop it into any ROS 2
workspace on this hardware and subscribe.

- ROS 2 **Jazzy**
- Backends: **cartographer** or **rtabmap**, chosen at launch
- A runnable Docker demo lives in
  [`xgo_localization_demo`](https://github.com/luckyTamme/xgo_localization_demo)

---

## Frame contract

```
map --(SLAM backend)--> odom --(this node)--> base_link --(static)--> base_laser
```

The `map` origin is the pose at startup, so **pose in `map` is literally "relative
to start"**.

Ownership is split so the two backends are interchangeable:

- This node owns `odom -> base_link`, unconditionally, from the first tick.
- The backend owns `map -> odom`, and nothing else.
- The static `base_link -> base_laser` is published here too (see
  [Laser mount](#laser-mount)).

Because the odometry layer never gaps, a backend that stalls, dies, or has not
converged yet does not stop the pose. `map -> odom` simply goes stale while
odometry keeps integrating: the node reports `DEGRADED` on `/diagnostics` and
keeps publishing. Dead reckoning is not as good as a corrected pose, but it is
always there, and the covariance says which one you are getting.

---

## Topics

Guaranteed, identical for both backends:

| topic | type | frame | meaning |
|---|---|---|---|
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | global pose, relative to start |
| `/localization/odom` | `nav_msgs/Odometry` | `odom` | continuous, drifting, carries twist |
| `/localization/map` | `nav_msgs/OccupancyGrid` | `map` | the active backend's grid |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | — | backend state and staleness |

Want velocity, take `odom`. Want global pose, take `pose`. There is deliberately
no map-frame `Odometry`, which would only duplicate `pose`.

`/diagnostics` stays at the root on purpose — it is a shared bus and standard
tooling subscribes to it absolutely. This node identifies itself inside the
message as `localization: backend`. `/tf` and `/tf_static` are at the root too,
which needs no special handling: tf2 uses those absolute names directly, so a
namespace never applies to them.

The namespace comes from the launch file. Running the node with bare `ros2 run`
puts `pose` and `odom` at the root instead — fine for debugging, but the launch
file is the supported entry point.

### Backend-specific extras

Passed through unchanged under the backend's own namespace, not unified. Check
`/diagnostics` if you need to know which backend is running.

| cartographer | rtabmap |
|---|---|
| `/localization/cartographer/submap_list` | `/localization/rtabmap/grid_prob_map` |
| `.../scan_matched_points2` | `.../cloud_map`, `.../cloud_obstacles`, `.../cloud_ground` |
| `.../trajectory_node_list`, `.../constraint_list` | `.../mapData`, `.../mapGraph` |
| `.../tracked_pose` | `.../info`, `.../labels`, `.../octomap_grid` |
| services `write_state`, `finish_trajectory` | services `reset`, `save_map` |

One behavioural difference worth knowing: **rtabmap persists a database between
runs**; cartographer starts empty every launch. The launch file passes
`--delete_db_on_start`, so the default is a fresh map either way.

---

## Inputs

| topic | type | notes |
|---|---|---|
| `/xgo/applied_vel` | `geometry_msgs/TwistStamped` | the clamped body velocity the driver actually applied |
| `/imu/data` | `sensor_msgs/Imu` | **orientation only** — see below |
| `/scan` | `sensor_msgs/LaserScan` | consumed by the backend, frame `base_laser` |

All three topic names are parameters, so the node is not welded to one driver.

The XGO firmware populates the IMU's `orientation` and nothing else:
`angular_velocity` and `linear_acceleration` are zero and flagged invalid
(`covariance[0] = -1`). This node reads orientation exclusively, and the
cartographer config leaves `use_imu_data` off for the same reason.

---

## Usage

```bash
ros2 launch xgo_localization localization.launch.py backend:=cartographer
ros2 launch xgo_localization localization.launch.py backend:=rtabmap
```

Replaying a bag? The clock has to match, on this node *and* the backend:

```bash
ros2 launch xgo_localization localization.launch.py backend:=rtabmap use_sim_time:=true
ros2 bag play <bag> --clock
```

Launch arguments: `backend`, `use_sim_time`, `namespace` (give it bare, no
leading slash), `scan_topic`, `params_file`, `rtabmap_params_file`.

### Check that it works

```bash
ros2 topic echo /localization/pose --once
ros2 topic echo /diagnostics --once
ros2 run tf2_ros tf2_echo map base_link
```

A healthy stack looks like this — `frame_id: map`, and a covariance of `0.01`
on x/y once the backend has converged:

```yaml
header:
  frame_id: map
pose:
  pose:
    position: {x: 2.82, y: 4.51, z: 0.0}
```

and on `/diagnostics`:

```yaml
level: 0                                  # OK
name: 'localization: backend'
message: map->odom fresh (0.03 s old)
```

`level: 1` (WARN) with `no map->odom correction yet` is expected for the first
few seconds while the backend converges. It should not persist.

### Consuming the pose

Nothing custom to build — it is a stock `PoseWithCovarianceStamped`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class WhereAmI(Node):
    def __init__(self):
        super().__init__('where_am_i')
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose', self.on_pose, 10)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        # Metres from where the robot started. x forward, y left.
        self.get_logger().info(f'{p.x:+.2f}, {p.y:+.2f}')
```

Pair it with `/diagnostics` if the decision matters: a pose published while the
backend is stale is dead-reckoned, and the covariance jumps accordingly.

---

## Health and diagnostics

The node publishes one `DiagnosticStatus` named `localization: backend` on
`/diagnostics`, at the pose rate.

| level | when | what it means |
|---|---|---|
| `OK` (0) | backend correcting | globally referenced pose |
| `WARN` (1) | no fresh `map -> odom` | still publishing, dead-reckoned from odometry |
| `ERROR` (2) | no IMU within `input_timeout_s` | odometry is **not running**; the pose is frozen |

The `ERROR` level is the one to alarm on. `WARN` means degraded but moving;
`ERROR` means the pose is a stale constant.

| key | meaning |
|---|---|
| `state` | `healthy` or `degraded` |
| `map_to_odom_age_s` | age of the newest correction, or `never`. **Signed** — a negative value means the backend stamps ahead of us, which is normal for rtabmap. A large negative value means a clock mismatch. |
| `ever_corrected` | has the backend ever produced a correction |
| `imu_age_s` | seconds since the last IMU message, or `never` |
| `applied_vel_age_s` | seconds since the last velocity message, or `never` |
| `pose_x`, `pose_y`, `pose_yaw_deg` | the pose just published |

---

## Troubleshooting

| symptom | likely cause | fix |
|---|---|---|
| `/diagnostics` stuck at `ERROR`, `imu_age_s: never` | no IMU arriving — wrong topic, or the driver is not running | check `imu_topic`; confirm `ros2 topic hz /imu/data` |
| Pose never leaves `degraded`, map stays empty | the backend is getting no scans | check `scan_topic` and that `/scan` is on frame `base_laser` |
| Pose barely moves while the robot drives | `applied_vel_age_s` growing — no velocity source | confirm the `xgo_ros` fork is running and publishing `/xgo/applied_vel` |
| Everything looks right but there is no `map` frame | backend has not converged yet | expected at startup; if it persists, the backend is not publishing `map -> odom` |
| Replay: pose jumps or diagnostics say "different clocks" | `use_sim_time` mismatch | pass `use_sim_time:=true` **and** `ros2 bag play --clock` |
| Pose drifts steadily to one side | laser mount not level | re-measure `laser_transform`, especially roll and pitch |
| Topics at `/pose` instead of `/localization/pose` | node started with `ros2 run` | use the launch file; it supplies the namespace |
| A viewer reports `base_laser` missing but SLAM works fine | joined after the one-shot static transform | should self-heal within `static_tf_period_s`; if not, reconnect |

---

## Installation

```bash
git clone https://github.com/luckyTamme/xgo_localization src/xgo_localization
rosdep install --from-paths src -y
colcon build
```

`rosdep` pulls both backends, so `backend:=` can switch without a rebuild.

### Runtime prerequisite

This node needs a source of **applied body velocity** as `TwistStamped`. On the
XGO2 that is [`luckyTamme/xgo_ros`](https://github.com/luckyTamme/xgo_ros),
branch `humble-m5.1.1-fix` — upstream `sskorol/xgo_ros` does not publish it.
Upstream relies on the firmware's autofeedback streaming, which XGO Mini 2
firmware **M-5.1.1** removed, so the upstream driver simply hangs. The fork polls
registers instead and adds `/xgo/applied_vel`.

It is not a `package.xml` dependency because it is not in rosdistro; declaring it
would break `rosdep install` for everyone.

Watch the names: the repository is `xgo_ros`, the package inside it is
`xgo2_ros`, and the executable is `xgo2_ros_node`.

The driver's own configuration matters too. `/xgo/applied_vel` reports the
*clamped* command, so the driver's velocity limits have to be set correctly for
this platform or the reported velocity will not match what the robot did.

**No driver is needed for replay** — a bag supplies all three inputs, so the
whole stack runs with no XGO hardware present.

---

## Laser mount

The LiDAR is a retrofit. No stock robot description contains it, and nothing else
in the stack publishes `base_link -> base_laser`, so this node does.

```yaml
laser_transform: [0.0, 0.0, 0.18, 0.0, 0.0, 0.0]   # x y z roll pitch yaw
```

**Measure it per robot.** A few millimetres of height error is harmless for 2D
SLAM. Pitch or roll error is not: it biases every range and shows up as
systematic drift. Check the mount is level, not just at the right height.

If this transform is missing entirely, SLAM backends drop every scan on an
unknown frame and produce an empty map with no error message — which is why it is
published unconditionally rather than left to the integrator.

It is also **re-sent every few seconds**, not just once. A latched `/tf_static`
is supposed to reach late subscribers on its own, but that only holds if every
hop preserves transient-local durability — one intermediary negotiating down to
volatile is enough for a consumer that connects mid-run to never learn where the
LiDAR is. Anything attaching to a running system is affected, viewers and
`ros2 bag record` alike.

---

## Parameters

See [`config/localization.yaml`](config/localization.yaml) for the full set with
comments. The ones worth knowing:

| parameter | default | meaning |
|---|---|---|
| `velocity_topic` | `/xgo/applied_vel` | applied body velocity input |
| `imu_topic` | `/imu/data` | orientation input |
| `map_frame` / `odom_frame` / `base_frame` / `laser_frame` | `map` / `odom` / `base_link` / `base_laser` | frame names, for running two instances |
| `laser_transform` | `[0.0, 0.0, 0.18, 0.0, 0.0, 0.0]` | `base_link -> base_laser`, measure per robot |
| `static_tf_period_s` | `5.0` | re-send the static transform this often, so late joiners still get it. `0` publishes once |
| `input_timeout_s` | `2.0` | no IMU for this long reports `ERROR` |
| `avodom.use_imu_heading` | `true` | heading from the IMU; false dead-reckons it instead |
| `avodom.max_dt_s` | `0.5` | skip integration steps longer than this |
| `avodom.scale_vx` / `scale_vy` | `1.0` | for platforms whose velocity is not metric |
| `arbiter.publish_rate_hz` | `20.0` | pose rate, independent of the backend |
| `arbiter.stale_after_s` | `1.0` | no correction newer than this means `DEGRADED` |
| `arbiter.healthy_xy_var` / `healthy_yaw_var` | `0.01` / `0.02` | covariance while corrected |
| `arbiter.degraded_xy_var` / `degraded_yaw_var` | `1.0` / `0.5` | covariance while dead-reckoning |

The `laser_transform` values must be written as floats. `[0, 0, 0.18, 0, 0, 0]`
mixes ints and doubles and the parameter parser rejects it.

Covariance is two fixed states, healthy and degraded, rather than a growth
curve. TF carries no covariance to propagate, so a smooth model would be invented
precision. Treat the numbers as nominal.

---

## Limitations

- **2D only.** Planar pose; z, roll and pitch are not estimated.
- **The `map` frame appears only once the backend converges.** Until then the
  pose topic flows (composed against an identity correction, flagged
  `DEGRADED`), but there is no `map` frame in TF, so RViz cannot render it yet
  and `map -> base_link` lookups fail. Publishing a placeholder was rejected
  deliberately: two publishers on one TF edge is worse than a missing edge.
- **Cartographer needs odometry TF before its first scan.** With
  `provide_odom_frame=false` it publishes nothing at all if it cannot look up
  `odom -> base_link` at the scan timestamp. The launch file starts this node
  first, so this only bites if the IMU is silent at startup — which
  `/diagnostics` reports as `ERROR`.
- **Needs a commanded-velocity source.** There is no wheel or joint odometry
  fallback; without `/xgo/applied_vel` the node has no motion prior.
- **Dead reckoning drifts.** In `DEGRADED` the pose stays continuous but has no
  global reference. Check `/diagnostics` before trusting it for anything
  long-running.
- **Loop closure can jump.** `map -> odom` is corrected discretely, so a global
  pose can step. Consumers needing smooth motion should use `odom`.
