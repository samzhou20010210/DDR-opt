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
