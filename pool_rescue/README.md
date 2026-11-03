# pool_rescue

Pool rescue simulation on top of DDR-opt: one annotated parameter file, interface messages, an adapter between
DDR-opt and those interfaces, and a dedicated launch file. The stock example (`plan_manager/launch/planner_sim.launch`)
is unchanged and still runs on its own.

```bash
roslaunch pool_rescue pool_sim.launch                      # approach mode, rviz, startup check
roslaunch pool_rescue pool_sim.launch mode:=rescue gui:=false
python3 pool_rescue/scripts/gen_ddr_params.py              # after editing config/pool_rescue.yaml
python3 pool_rescue/scripts/check_params.py                # offline: schema, sanity, generated files current
rosrun pool_rescue check_params.py --live --mode approach  # against a running stack (also run by the launch file)
```

There are two independent stacks. **Never launch both at once**: each one has its own single publisher of
`/pool/robot/odom` and of the `world→base_link` TF.

| Launch | What runs | Owner of `/pool/robot/odom` and `world→base_link` |
|---|---|---|
| `pool_hull_sim.launch` | `pool_env` + `hull_sim` (no planner) | `hull_sim` |
| `pool_sim.launch` | DDR-opt planner, MPC, legacy simulator, EKF + `ddr_adapter` | `ddr_adapter` |

## 0. Pool environment and hull simulator (`pool_hull_sim.launch`)

```bash
roslaunch pool_rescue pool_hull_sim.launch [mode:=approach|rescue] [gui:=false] [reverse_enabled:=true|false]
rostopic pub -r 20 /pool/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.3}}'
rosservice call /hull_sim/reset                 # back to standby, at rest
rosrun pool_rescue hull_tests.py --out /tmp/hull_tests [--tests straight stop turn arc reverse wall timeout]
```

**`pool_env`** (`scripts/pool_env.py`) reads `/pool/pool/*`, `/pool/standby/*` and `/pool/extraction_zone/*` and
publishes, latched:
- `/pool/env/boundary` (`PolygonStamped`): the water rectangle [0, L] × [0, W] in `world`.
- `/pool/env/markers` (`MarkerArray`): water, walls (drawn just outside the boundary), boundary line, standby
  outline, extraction zone, labels.

**`hull_sim`** (`src/hull_sim.cpp`) reads `/pool/mode` and `/pool/<group>/<name>/value`.

| | |
|---|---|
| Input | `/pool/cmd_vel` (`Twist`): `linear.x` = target speed [m/s], `angular.z` = target yaw rate [rad/s] |
| Latency | each command takes effect after `control.latency` |
| Limits | target clamped to [−max_reverse_speed, max_forward_speed] and ±max_yaw_rate of the active mode; reverse clamps to 0 if `vessel.reverse_enabled` is false |
| Speed | rises at `max_acceleration` and falls at `max_deceleration`; a sign change decelerates to 0 first |
| Yaw rate | changes at `max_yaw_accel` |
| Integration | unicycle at `simulation.hull_sim_rate` (trapezoidal v/ω, midpoint heading); no sway, drag or currents |
| Lost link | no command for `control.cmd_timeout` → target (0, 0) |
| Walls | the hull rectangle (`vessel.length × width`) must stay inside the pool. A step that would push a corner out is rejected (pose kept, v = ω = 0, `/pool/robot/wall_contact` = true). Contact stays latched while the same target keeps arriving; any other target may move the hull again |
| Startup | the node exits with an error unless the whole hull at the standby pose is ≥ `safety.wall_clearance` from every wall |
| Output | `/pool/robot/odom` (`Odometry`, stamped, `world`→`base_link`, body-frame twist), TF `world→base_link`, `/pool/hull_sim/target` (`TwistStamped`: target after latency + clamping), `/pool/robot/wall_contact` (`Bool`, latched), `/pool/robot/trajectory` (`Path`, every `simulation.trajectory_period`), `/pool/robot/hull_markers` (outline + heading arrow in `base_link`) |
| Service | `/hull_sim/reset` (`std_srvs/Empty`) |

RViz config: `rviz/pool.rviz`, a top-down view of the whole pool showing the Pool markers, Trajectory, Hull and TF.

**Response tests** (`scripts/hull_tests.py`): each test resets the vessel, sends a timed `/pool/cmd_vel` schedule
at 50 Hz, records odometry, and compares the measured response with the prediction from the config:

| Test | Measured |
|---|---|
| `straight` | latency, acceleration, steady speed (commanded at 1.5 × limit, so clamping shows), lateral and heading drift |
| `stop` | latency, deceleration, stopping time, stopping distance, creep after the stop |
| `turn` | latency, yaw acceleration and deceleration, steady yaw rate, position drift while turning in place |
| `arc` | steady turn radius = v / ω |
| `reverse` | reverse speed, acceleration and direction; with reverse disabled, no backward motion |
| `wall` | hull never outside the pool, contact reported, no motion while pushing, recovery by reversing |
| `timeout` | stop after latency + cmd_timeout without commands |

Outputs: `<test>.csv`, `<test>.png` and `report.json`; exit code 0 means all checks passed. Recorded results are in
`test_results/hull_sim/`.

## 1. Parameters

`config/pool_rescue.yaml` is the **only** file to edit. Every leaf is `{value, unit, source}`; `source` is
`stock: …` (kept from DDR-opt), `standard: …` or `assumption: …` (to be confirmed on the real vessel).

| Group | Contents |
|---|---|
| `frames` | `world`, `base_link` names |
| `pool` | length 25 m, width 12.5 m, wall thickness in map, map resolution |
| `standby` | start pose near the (0, 0) corner |
| `vessel` | length, width, height, thruster separation, coupled v/ω limits |
| `control` | latency, MPC dt, control rate, replan period, planner time budget |
| `safety` | wall clearance, person clearance |
| `sensing`, `simulation` | detection range, perspective sensing, simulator noise |
| `modes.approach`, `modes.rescue` | forward/reverse speed, accel/decel, yaw rate/accel, centripetal accel, plus mode-specific (standoff distance, final approach speed / docking tolerances) |
| `extraction_zone` | default hand-over zone |

`scripts/gen_ddr_params.py` maps these onto the flat keys DDR-opt nodes read, writing
`config/generated/<mode>/<node>.yaml` (committed, never hand-edited). Derived values:

| Generated key | Rule |
|---|---|
| `max_vel` / `min_vel` | forward speed / −reverse speed |
| `max_acc` | min(acceleration, deceleration): DDR-opt has one symmetric limit |
| `checkpoint` | 4 corners + 4 edge midpoints of the vessel footprint (body frame) |
| `safeDis` | wall clearance (applied at every checkpoint) |
| `jps_safe_dis` | vessel width / 2 + wall clearance (JPS plans for the centre point) |
| `finalMinSafeDis` | vessel width / 2 (final check only looks at the centre) |
| `delay_num` (MPC) | round(latency / mpc_dt) |
| `ICR_yl` / `ICR_yr` | ± thruster separation / 2 |
| `map_size`, `map_box` | pool rectangle and 4 explicit wall boxes (`if_boundary_wall_: false`, see DEVLOG) |

In the launch file, each legacy node loads its stock yaml first and then its generated file, so the pool values win.
Values that aren't in the central file (optimizer weights, MPC Q/R/horizon, EKF noise, laser model) keep their stock
values.

**Modes.** The launch argument `mode` picks the generated directory. DDR-opt nodes read their limits only at startup,
so switching between approach and rescue currently means relaunching. Switching at runtime is future work for the
mission layer.

**`/mpc` namespace.** `mpc_controller` reads only `/mpc/<key>`, and uses −1 for any key that's missing. The node
must therefore be named `mpc`, and its parameters must be loaded inside that node tag. The live check verifies all
16 keys it reads.

## 2. Frames and TF

| Frame | Definition | Single publisher |
|---|---|---|
| `world` | origin at the pool corner next to the standby pose; x along the length, y along the width, z up | root (no parent) |
| `base_link` | vessel centre; x forward, y left | `world→base_link`: `ddr_adapter` (EKF estimate) |
| `laser` | simulated sensor at the true pose (simulation only) | `world→laser`: `laser_simulator` (stock) |

No other node publishes TF. The live check subscribes to `/tf` and fails if any frame has more than one parent or
publisher.

## 3. Topics

Canonical interfaces (new code should use only these):

| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| `/pool/robot/odom` | `nav_msgs/Odometry` | `ddr_adapter` | estimated pose (EKF); frame `world`, child `base_link`; twist in body frame (from the simulator, since the EKF doesn't estimate velocity) |
| `/pool/robot/odom_truth` | `nav_msgs/Odometry` | `ddr_adapter` | ground truth from the simulator (for evaluation only) |
| `/pool/cmd_vel` | `geometry_msgs/Twist` | `ddr_adapter` (from MPC) | desired planar motion: `linear.x` [m/s] forward, `angular.z` [rad/s] yaw rate; other fields 0 |
| `/pool/persons` | `pool_rescue/PersonArray` | *(none yet)* | tracked persons |
| `/pool/rescue_task` | `pool_rescue/RescueTask` | *(none yet)* | current rescue order |

Messages (`msg/`, all in the `world` frame):

- **`Person`**: `header` (stamp = observation time), `id` (unique, stable), `position` [m], `velocity` [m/s],
  `yaw` [rad], `length`, `width` [m] (occupied footprint).
- **`PersonArray`**: `header`, `persons[]`.
- **`ExtractionZone`**: `name`, `pose` (`Pose2D`: centre [m] and required final heading [rad]), `length`, `width` [m].
- **`RescueTask`**: `header`, `task_id`, `person_id` (matches `Person.id`), `extraction_zone`.

Legacy DDR-opt topics, unchanged and internal:

| Topic | Type | Path |
|---|---|---|
| `/planner/traj_poly` | `carstatemsgs/Polynome` | global_planning → mpc |
| `/pool/internal/mpc_cmd` | `carstatemsgs/CarState` | mpc `~cmd` (remapped) → ddr_adapter |
| `/simulation/PoseSub` | `carstatemsgs/CarState` | ddr_adapter (after `latency`) → simulator. `global_planning` also advertises this topic but never publishes on it |
| `/ICREKF/EKF_XYTheta` | `nav_msgs/Odometry` | icrekf → planner, mpc, ddr_adapter |
| `/simulation/PosePub` | `carstatemsgs/CarState` | simulator ground truth → laser_simulator, ddr_adapter |

## 4. Adaptation layer (`src/ddr_adapter.cpp`)

The adapter is the only place that converts between DDR-opt messages and the interfaces above; no legacy code was
changed. Its path is `mpc_cmd (CarState) → /pool/cmd_vel (Twist) → [latency queue] → /simulation/PoseSub`.
Because the latency is applied on the vessel side, a real vessel driver would subscribe to `/pool/cmd_vel` in place
of the simulator. The MPC compensates for the latency with `delay_num`.

The adapter is C++ rather than Python because the simulator subscribes with `TransportHints().unreliable()`
(UDPROS), and rospy publishers can't connect to such a subscriber.
