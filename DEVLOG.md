# Development Log — Pool Rescue Robot (built on DDR-opt)

Goal: simulate a 2D pool rescue robot (water surface, differential thrust from left/right thrusters) on top of
DDR-opt. Mission (as given so far): start from a standby position near a pool corner, then … *(mission spec was
truncated — see Open questions)*.

This log records the **baseline** (upstream DDR-opt as of commit `ad29c7d`) and every change made on top of it.
Paths are relative to `src/DDR-opt-main/`.

---

## 1. Baseline: how DDR-opt works

### 1.1 Data flow

```
global_map_node ──/global_map/gridmap (PointCloud2, ground truth)──> laser_sim_node
simulator ──/simulation/PosePub (CarState, ground truth)──────────────> laser_sim_node (also TF world→laser)
laser_sim_node ──/laser_simulator/local_pointcloud──> global_planning (SDFmap builds occupancy + ESDF from it)
simulator ──/simulation/PoseOdomPub (Odometry, 10 Hz)───┐
simulator ──/simulation/ControlPub (wheel speeds)───────┼──> icrekf ──/ICREKF/EKF_XYTheta (Odometry, 100 Hz)
simulator ──/simulation/SimulatedCarState───────────────┘           └─/ICREKF/EKF_ICR (yr, yl, xv)
/move_base_simple/goal ──> global_planning ──/planner/traj_poly (Polynome)──> mpc | nmpc
                                         └──/planner/emergency_stop (Bool)──> mpc | nmpc
mpc  ──/simulation/PoseSub   (CarState: v, ω)──────────> simulator
nmpc ──/simulation/ControlSub (CarControl: wheel speeds)─> simulator
```

### 1.2 Trajectory representation (the core idea)

- The trajectory is **not** in x-y. It is a MINCO piecewise quintic `Trajectory<5,2>` over the flat output
  σ(t) = (θ(t), s(t)) — **dimension 0 = yaw, dimension 1 = arc length**. So `getPos(t) = (θ, s)`,
  `getVel(t) = (ω, v)`, `getAcc(t) = (α, a)`.
- x-y is recovered by integration: ẋ = ṡ cosθ, ẏ = ṡ sinθ (standard diff). Non-standard diff (`if_standard_diff:
  false`) adds the ICR lateral slip term: ẋ += ω·xv·sinθ, ẏ −= ω·xv·cosθ. Integration everywhere uses Simpson's
  rule (optimizer, `check_final_collision`, `get_the_predicted_state`, controllers' `TrajAnal::getSeq`).
- Consequence: every consumer must know the **start pose** (`Polynome.start_position`) and integrate; errors grow
  along the trajectory.
- `Polynome` msg carries: `init_p/v/a`, `tail_p/v/a` (each (θ,s) in x/y fields), `innerpoints` (θ,s waypoints),
  `t_pts` (segment durations), `start_position` (x, y, yaw), `ICR` (x=yl, y=yr, z=xv), `traj_start_time`.

### 1.3 Planner state machine (`plan_manager/include/plan_manager/plan_manager.hpp`)

- States: `INIT → IDLE → PLANNING ⇄ REPLAN → GOINGTOGOAL`, plus `EMERGENCY_STOP`. 1 ms timer `MainThread`.
- Every tick: if ESDF at current position < 0 → publish emergency stop, state `EMERGENCY_STOP` (terminal; nothing
  leaves it).
- `IDLE` + goal → plan from current EKF pose with v = ω = 0 (odometry callback zeroes VAJ/OAJ).
- `PLANNING/REPLAN` and `replan_time` elapsed → if near goal (`dist² + 0.02·|Δyaw| < 1`) or remaining duration
  < `max_replan_time` → `GOINGTOGOAL`; else predict the state at `now + max_replan_time` on the current trajectory
  and replan from there (trajectory continuity).
- Goals are **ignored unless state is IDLE** (`"Haven't reached the goal yet!!"`). Returns to IDLE when the
  trajectory duration has elapsed.
- `if_fix_final: true` replaces any clicked goal with `final_x/y/yaw`.

### 1.4 Front end (`front_end/src/jps_planner/`)

1. JPS grid search on the ESDF-inflated grid (`jps_safe_dis`), 8-connected.
2. `removeCornerPts` shortcut-smooths the path (line-of-sight checks).
3. `getSampleTraj`: path → sequence of (x, y, θ, Δθ, Δs) nodes; **turn-in-place at every corner** then drive
   straight. Initial heading always forward along the path.
4. `getTrajsWithTime`: trapezoidal velocity profile on weighted length (`jps_distance_weight·s + jps_yaw_weight·|θ|`),
   sampled every `timeResolution` (≥ `mintrajNum` segments) → `FlatTrajData` (θ,s waypoints + x-y positions).
   Path is cut at `trajCutLength` (600 m ⇒ effectively never cut).

### 1.5 Back end (`back_end/src/optimizer.cpp`, `MSPlanner`)

Decision vector: 2(N−1) inner (θ,s) points, final s (free), N virtual times (T = smooth map of τ, `VirtualT2RealT`).

- **Stage 1 — pre-processing** (`costFunctionCallbackPath`): kinematic penalties + pull integrated waypoint x-y to the
  JPS positions (`bigpath_sdf_weight`). No ESDF.
- **Stage 2 — main** (`costFunctionCallback` + `attachPenaltyFunctional`), L-BFGS inside an Augmented-Lagrangian loop
  on the final x-y equality (`EqualRho/Lambda/Gamma/Tolerance`; `Cut*` variants when the path is cut). Terms:
  - MINCO energy (`energyWeights` [yaw, s]) + `time_weight·ΣT`
  - |a| ≤ `max_acc`, |α| ≤ `max_domega`
  - velocity: either |v| ≤ max_vel and |ω| ≤ max_omega (`if_directly_constrain_v_omega: true`), or the wheel-limit
    diamond `max_vel·|ω| + max_omega·v ≤ max_vel·max_omega` (+ the `min_vel` reverse side), weight `moment_weight`
  - |v·ω| ≤ `max_centripetal_acc`
  - collision: for every `checkpoint` (body-frame points, default single point [0,0]),
    `safeDis − ESDF > 0` penalised; gradients chained back through the Simpson integration
  - segment-time balance within [`mean_time_lowBound`, `mean_time_uppBound`] × mean
- All penalties use `positiveSmoothedL1` (`smoothingFactor`).
- After optimization `check_final_collision` (ESDF ≥ `finalMinSafeDis`); on failure retries up to
  `safeReplanMaxTime` with `time_weight *= 0.75`. `safeDis` is clamped to 0.85× the start-point clearance.

### 1.6 Map (`utils/plan_env`, `SDFmap`)

- Global grid `global_{x,y}_{lower,upper}` at `gridmap_interval`, built online from the laser cloud (TF lookup
  world←cloud frame). Perspective mode marks the whole detection window free then stamps cloud points occupied.
  Non-perspective mode: log-odds raycasting; occupied cells are never cleared.
- ESDF (signed, Felzenszwalb) recomputed at 20 Hz inside the detection window.
- **`getDistanceReal` returns 10000 outside the global bounds** → map edges are not obstacles unless walls exist in
  the map. **Unknown cells count as free** in the ESDF.

### 1.7 Simulator (`utils/simulator`)

- Pure **kinematic** unicycle (+ optional ICR lateral velocity `vy = −ω·xv`), 500 Hz propagation; v/ω rate-limited
  by `max_acc`/`max_domega`; optional multiplicative velocity noise. No inertia, drag, or disturbances.
- Two input modes: `/simulation/PoseSub` (v, ω — overwrites the state velocity directly) or
  `/simulation/ControlSub` (wheel speeds → v, ω, vy via ICR `yl, yr, xv`).
- `global_map_node`: `Map_input_method` 1 = yaml `map_box` [cx, cy, sx, sy, yaw]×n, 2 = random boxes, 3 = png.
  Boundary walls are added at the map edges (param read as `if_boundary_wall_`, default true — note the launch files
  set `if_boundary_wall` without the trailing underscore, so the default applies).

### 1.8 Controllers

- `mpc_controller`: linearized unicycle MPC (OSQP), state (x, y, θ), inputs (v, ω), horizon `predict_steps`×`dt`.
  Reads params from the absolute `/mpc/...` namespace ⇒ **the node must be named `mpc`**. Reference via `TrajAnal`
  (standard diff only, ignores ICR). Holds zero command 1 s after trajectory end.
- `nmpc_controller`: ACADO NMPC with the ICR model (inputs = left/right wheel speeds `vl, vr`), ICR updated online from
  `/ICREKF/EKF_ICR`. Model: ẋ = (vr·yl − vl·yr)/(yl−yr)·cosψ + (vr−vl)·xv/(yl−yr)·sinψ, etc.
- `icrekf`: 6-state EKF (x, y, ψ, yr, yl, xv); prediction from wheel speeds, correction from the (ground-truth) odom.

---

## 2. Assumptions carried over that matter for a boat

| Baseline assumption | Why it matters on water |
|---|---|
| Kinematic, no-slip robot (velocity commands applied almost instantly) | Boats have inertia, linear+quadratic drag, sway drift and yaw overshoot; the planner's `max_acc`/`max_domega` must be set from thruster/drag limits. |
| Lateral slip only through the constant ICR `xv` | Sway is velocity-dependent; `xv` may be a first approximation (`if_standard_diff: false`). |
| Point robot + `safeDis` | A rescue robot has a real hull; use several `checkpoint`s and a pool-wall clearance. |
| Map edges are free space | Pool walls must be explicit obstacles (boundary walls / map boxes). |
| Planning only from IDLE with v = ω = 0 | A rescue mission needs goal changes on the fly (moving victim) and replanning from a moving state. |
| `EMERGENCY_STOP` is terminal; JPS failure is not detected (`findJPSRoad` always returns true) | Mission-level recovery logic needed. |
| Laser sensing of static obstacles | Victim detection / swimmers as dynamic obstacles are not modelled. |

---

## 3. Execution

### 3.1 Verified environment (2026-11-03)

Ubuntu 20.04.6, ROS Noetic, catkin_tools 0.9.4 (devel layout `linked`, no extra CMake args), Python 3 + rospy,
OSQP + OSQP-Eigen 0.8.1 in `/usr/local`, `ros-noetic-tf2-sensor-msgs`, `pcl_ros`. **plotjuggler is NOT installed**
(the stock launch files start it unconditionally, so that node fails to start in GUI runs — harmless). No Xvfb.
A clean build from scratch (fresh workspace) takes ~70 s at `-j8`: 10/10 packages, 16 benign compiler warnings.

### 3.2 Interactive (GUI)

```bash
cd ~/main/pr_robot/DDRopt_ws
source /opt/ros/noetic/setup.bash
catkin build && source devel/setup.bash
roslaunch plan_manager planner_sim.launch      # v/ω + linear MPC
roslaunch plan_manager planner_nmpc.launch     # wheel speeds + ACADO NMPC (ICR model)
# set goal with rviz "2D Nav Goal"
```

### 3.3 Headless end-to-end check

```bash
src/DDR-opt-main/scripts/run_headless_sim.sh                 # build + run + verify, seed 2, goal (3.7, 0.3, 3.14)
src/DDR-opt-main/scripts/run_headless_sim.sh --no-build --seed 4 --goal 0 5 1.57 --out /tmp/myrun
```

- Launches `planner_sim.launch gui:=false map_seed:=N` on a private master (port 11411, `DISPLAY` unset), waits for
  EKF odom + 3 s map build, publishes one goal, records until the trajectory ends + 2 s.
- `scripts/smoke_test.py` writes `report.json`, `truth.csv` (ground-truth x, y, yaw, v, ω), `cmd.csv` (MPC v, ω),
  `planned_path.csv` (from `/visualizer/mincoPath`), plus `launch.log` and ROS logs into the out dir. Exit 0 = PASS.
- PASS needs: trajectory published, no emergency stop, robot moved, final position error < 0.3 m, stopped,
  commands within ±3 m/s / ±3 rad/s, max cross-track error to the planned path < 0.3 m.
- A one-off GUI launch with a fixed map: `roslaunch plan_manager planner_sim.launch map_seed:=2`.

### 3.4 Baseline results (`planner_sim.launch`, goal (3.7, 0.3, 3.14))

| Seed | Result | Notes |
|---|---|---|
| 1 | FAIL (expected) | JPS: no path to goal (random boxes only keep clear of the start). Before the fix: **`global_planning` segfaulted**. After: goal dropped, node stays up; a follow-up goal (−5, −0.5, 0) in the same session PASSes |
| 2 | PASS | 19 pieces, 7.62 s, 11.58 m; plan 24 ms (JPS 0.4 ms, pre-process 10 ms); min clearance 0.31 m; cmd v ∈ [−0.30, 2.78], ω ∈ [−0.88, 2.57]; final err 1.6 cm / 0.003 rad; max tracking err 2.6 cm |
| 3 | FAIL (expected) | same as seed 1 |
| 4 | PASS | |

Repeat runs of seed 2 give bit-identical plans; final pose varies by ~1 cm because the simulator adds velocity
noise (`if_add_noise: true`).

**Fixed — JPS failure crash:** `PlanManager::findJPSRoad()` ignored the return value of `JPSPlanner::plan()`; with
no path it continued on an empty path and the node segfaulted (`exit code -11`). Now it returns `false`, and
`MainThread` drops the goal and goes back to `IDLE` (instead of the dead-end `EMERGENCY_STOP`), so the next goal is
accepted. If the failure happens during a replan (robot already moving), it first publishes
`/planner/emergency_stop`; the MPC zeroes its command and resumes on the next trajectory. That replan branch is
**not exercised** by the headless test (`planner_sim.launch` uses `replan_time: 5000`, so it never replans).

---

## 4. Change log

| Date | Change | Files |
|---|---|---|
| 2026-11-03 | Created this log: baseline analysis, interfaces, assumptions for the pool-rescue extension. No code changed. | `DEVLOG.md` |
| 2026-11-03 | Headless workflow: `gui` (default true) and `map_seed` (default −1) args in `planner_sim.launch` — defaults keep stock behaviour; headless runner + automated goal/trajectory/control/motion check; baseline results recorded. | `plan_manager/launch/planner_sim.launch`, `scripts/run_headless_sim.sh`, `scripts/smoke_test.py`, `DEVLOG.md` |
| 2026-11-03 | New package `pool_rescue`: central annotated config (49 params, value/unit/source), generator → per-node DDR-opt params for `approach`/`rescue`, messages (Person, PersonArray, ExtractionZone, RescueTask), C++ `ddr_adapter` (Twist/Odometry interfaces, latency, single `world→base_link` TF), `pool_sim.launch`, offline + live validator. Stock example untouched. See §5. | `pool_rescue/**`, `DEVLOG.md` |
| 2026-11-03 | Stage 3, DDR-opt navigation on the hull: `pool_nav.launch` (hull_sim + planner + MPC + map + laser + `nav_bridge`; legacy simulator and EKF off), circle collision model with one clearance derivation, `/planner/status` + explicit failure handling in `plan_manager.hpp`, lane ropes (visual only), `scenarios.yaml` + generated scenario maps, `nav_tests.py`, `run_nav_scenarios.sh`, live check `--stack nav`; `safety.wall_clearance` renamed `safety.margin`. See §7. | `plan_manager/include/plan_manager/plan_manager.hpp`, `pool_rescue/**`, `DEVLOG.md` |
| 2026-11-03 | Stage 2, pool environment + hull simulator: `pool_env` (boundary + RViz markers from config), C++ `hull_sim` (latency, asymmetric accel/decel, yaw accel, reverse switch, cmd timeout, wall contact, standby validity check, odom/TF/trajectory/hull markers, reset service), `pool_hull_sim.launch`, `rviz/pool.rviz`, `hull_tests.py`; 5 new central params; the generator gives the planner `min_vel = 0` when reverse is disabled. See §6. | `pool_rescue/**`, `DEVLOG.md` |
| 2026-11-03 | Fix planner segfault when JPS finds no path: `findJPSRoad()` returns false; `MainThread` drops the goal → `IDLE` (emergency stop first if replanning). Verified: seeds 1/3 no crash + recovery to a new goal; seeds 2/4 PASS, seed 2 plan identical. | `plan_manager/include/plan_manager/plan_manager.hpp`, `DEVLOG.md` |

---

## 5. Pool rescue configuration and interfaces (`pool_rescue/`)

Full interface spec: `pool_rescue/README.md`. Summary:

- **Single source of parameters:** `pool_rescue/config/pool_rescue.yaml`, where every value has a unit and a source
  (stock / standard / assumption). `scripts/gen_ddr_params.py` generates
  `config/generated/{approach,rescue}/<node>.yaml`. `pool_sim.launch mode:=approach|rescue` loads, for each legacy
  node, the stock yaml first and then the generated file.
- **Interfaces:** `/pool/robot/odom` (Odometry, estimate), `/pool/robot/odom_truth`, `/pool/cmd_vel` (Twist),
  `/pool/persons` (PersonArray), `/pool/rescue_task` (RescueTask). Frames: `world` (pool corner), `base_link`.
  TF owners: `world→base_link` = `ddr_adapter`, `world→laser` = `laser_simulator`.
- **Initial assumptions** (all marked `assumption:` in the yaml, to be confirmed):
  - pool 25 × 12.5 m
  - vessel 1.0 × 0.6 m
  - approach mode: 1.0 / −0.3 m/s, accel 0.5 / decel 0.4 m/s², 0.8 rad/s
  - rescue mode: about half of every approach limit
  - control latency 0.1 s
  - wall clearance 0.3 m

Legacy issues found while doing this, all handled without changing legacy code:
1. **Boundary walls are wrong on non-square maps.** `global_map_node` swaps the x and y extents of its automatic
   walls, which would leave a 12.5 m gap in each long wall of a 25 × 12.5 m pool. Also, the launch files set
   `if_boundary_wall`, but the code reads `if_boundary_wall_`. The pool config therefore sets
   `if_boundary_wall_: false` and generates the four walls as explicit `map_box` entries.
2. **`mpc_controller` silently uses −1 for missing `/mpc/<key>` params.** The node must be named `mpc`, and the live
   check verifies every key it reads.
3. **The simulator's command subscriber is UDP-only** (`TransportHints().unreliable()`), so rospy publishers can't
   reach it. That's why the adapter is written in C++.
4. **No `base_link` frame existed.** The only TF in the stock stack was `world→laser` from the laser simulator.

Validation (2026-11-03, headless, private master):
- `check_params.py`: offline 26/26 checks, live 52/52 checks in both modes. The live run covers:
  - generated values on every node
  - all 16 `/mpc/*` keys
  - `/pool/*` loaded
  - one publisher per TF frame
  - interface topic types and publishers
- Negative tests:
  - tampering with a generated file → offline FAIL
  - running the rescue-mode check against an approach-mode stack → 4 FAILs
- Drive from standby (1.2, 1.2) to (20, 8, 0) through the adapter with 0.1 s latency:

| Mode | Plan | v max | \|ω\| max | Final error | Max tracking error |
|---|---|---|---|---|---|
| approach | 30.8 s | 0.80 m/s | 0.52 rad/s | 1 mm | 3 mm |
| rescue | 58.6 s | 0.42 m/s | 0.40 rad/s | 3 mm | 2 mm |

Not done yet: nothing publishes persons or tasks; persons are not obstacles in the planner map (`person_clearance`
is defined but not used); mode switching requires a relaunch; the dynamics are still kinematic.

---

## 6. Stage 2: pool environment and hull simulator (no planner)

`roslaunch pool_rescue pool_hull_sim.launch`. Interfaces and model are described in `pool_rescue/README.md` §0.
The hull simulator is a new node and does not use DDR-opt's `simulator`.

New central parameters:
- `vessel.reverse_enabled` (true)
- `control.cmd_timeout` (0.5 s)
- `simulation.hull_sim_rate` (100 Hz)
- `simulation.trajectory_period` (0.1 s)
- `simulation.trajectory_max_points` (6000)

The validator now checks 54 annotated parameters.

**Model assumptions:**
- Kinematic unicycle with rate-limited v and ω, and a pure transport delay on commands.
- No sway, drag, inertia beyond the rate limits, waves or currents.
- Walls are a hard stop for the hull rectangle (no bounce).

**Recorded responses** (2026-11-03, headless; full data in `pool_rescue/test_results/hull_sim/`). Expected values
come from the config; all checks pass in both modes:

| Test | Quantity | approach: expected / measured | rescue: expected / measured |
|---|---|---|---|
| straight (cmd 1.5 × limit) | latency | 0.100 / 0.101 s | 0.100 / 0.106 s |
| | acceleration | 0.50 / 0.500 m/s² | 0.25 / 0.250 m/s² |
| | steady speed (clamped) | 1.00 / 1.000 m/s | 0.50 / 0.500 m/s |
| | lateral / heading drift | 0 / 0 | 0 / 0 |
| stop | latency | 0.100 / 0.107 s | 0.100 / 0.108 s |
| | deceleration | 0.40 / 0.400 m/s² | 0.25 / 0.250 m/s² |
| | stopping time (L + v/dec) | 2.600 / 2.597 s | 2.100 / 2.108 s |
| | stopping distance (v·L + v²/2dec) | 1.350 / 1.350 m | 0.550 / 0.550 m |
| | creep after stop | 0 / 0 | 0 / 0 |
| turn in place | latency | 0.100 / 0.108 s | 0.100 / 0.101 s |
| | yaw accel / decel | 1.00 / 1.000, 1.000 rad/s² | 0.50 / 0.500, 0.500 rad/s² |
| | steady yaw rate | 0.80 / 0.800 rad/s | 0.40 / 0.400 rad/s |
| | position drift | 0 / 0 | 0 / 0 |
| arc (v = ½ v_max, ω = ½ ω_max) | radius v/ω | 1.250 / 1.250 m | 1.250 / 1.250 m |
| reverse | speed / accel | −0.30 / −0.300 m/s, 0.50 / 0.500 m/s² | −0.20 / −0.200 m/s, 0.25 / 0.250 m/s² |
| reverse, disabled (approach) | backward speed / motion | 0 / 0, 0 / 0 | n/a |
| wall contact | min hull-wall margin / motion while pushing | ≥ 0 / 0.0009 m, 0 | ≥ 0 / 0.0006 m, 0 |
| | recovery by reversing | ≥ 0.05 / 0.51 m | ≥ 0.05 / 0.32 m |
| cmd timeout | decel starts after L + timeout | 0.600 / 0.582 s | 0.600 / 0.580 s |

- **Measured latency** is the configured 0.1 s plus up to one 10 ms simulation tick plus ROS transport.
- **The timeout fires about 20 ms early** relative to the time the test stopped publishing, because the last
  command left the test node up to one 20 ms publish period earlier.
- **Invalid standby is rejected:** at x = 0.3 m the node exits with
  `standby pose ... leaves only -0.2 m between hull and wall, need 0.3 m`.

Issues found and fixed during testing:
1. **Test bug:** the ramp-slope helper mixed the ramp-up with the later ramp-down in the turn and reverse tests,
   giving ≈ 0 rad/s². It now uses the first contiguous ramp only.
2. **Model bug:** at wall contact the hull chattered (about 0.03 m/s spikes for about 0.1 s) because each tick
   re-accelerated from zero. Contact is now latched while the same target keeps arriving.

**RViz:** `rviz/pool.rviz` was opened on the desktop and the RViz log shows no errors. All published visualization
data was checked headlessly:
- 6 environment marker groups in `world`, boundary polygon 25 × 12.5 m
- hull outline 1.0 × 0.6 m and heading arrow in `base_link`
- trajectory growing in `world`
- `world→base_link` resolvable

A screenshot check was abandoned, because without a compositor `xwd` also captures overlapping windows.

---

## 7. Stage 3: DDR-opt single-goal navigation on the pool hull

`roslaunch pool_rescue pool_nav.launch scenario:=<name>` and `pool_rescue/scripts/run_nav_scenarios.sh`. Design
details are in `pool_rescue/README.md` (section "Navigation").

- **Reused unchanged:** JPS front end, MINCO back end, MPC, SDF map, laser simulator.
- **Disabled:** the legacy `simulator` and `icrekf`. The planner and MPC take `hull_sim` ground truth on
  `/pool/robot/odom`.
- **Adaptation layer:** `nav_bridge` converts the MPC's CarState command to `/pool/cmd_vel` (Twist), and
  `hull_sim`'s odometry to the CarState sensor pose the laser simulator expects.
- **Lane ropes:** 4 ropes at y = 2.5 / 5 / 7.5 / 10 m (5 × 2.5 m lanes, World Aquatics lane width, 0.10 m floats).
  They appear in RViz and the plots only, never in the planner map.

**Collision model (circle).**
- r = 0.583 m (derived), m = 0.30 m.
- Planning (JPS and optimizer) uses r + m = 0.883 m; final verification uses r.
- The map is not inflated.
- The earlier 8-point footprint with `finalMinSafeDis = width/2` was dropped. The legacy final check only looks at
  the centre, so width/2 missed the hull's ends: exactly the "omission" the circle model avoids.

**DDR-opt code change (`plan_manager.hpp`).**
- A latched `/planner/status` topic.
- A `failPlanning()` helper used for both JPS and optimizer failures.
- Before, an optimizer failure just returned. The planner stayed in PLANNING and only reached idle again through a
  comparison with the uninitialized `Traj_total_time_` (now initialized). No trajectory was published even then,
  but nothing reported the failure.

**Recorded results** (2026-11-03, approach mode, headless; data in `pool_rescue/test_results/nav/<scenario>/`). The
live check passed 58/58 in every run.

| Scenario | Expect | Status sequence | Plan | Actual |
|---|---|---|---|---|
| obstacle_straight | success | GOAL_RECEIVED → TRAJECTORY_PUBLISHED → TRAJECTORY_COMPLETE | 20.42 m, 32.6 s, computed in 0.061 s | end error 4 mm / 0.000 rad; min hull clearance 0.545 m; min circle clearance 0.272 m (planned 0.271); tracking ≤ 2 mm |
| near_wall | success | same | 22.60 m, 34.1 s, 0.034 s | 1 mm; hull 0.700 m; circle 0.517 m; tracking ≤ 2 mm |
| near_wall_too_close | failure | GOAL_RECEIVED → FAILED_NO_PATH | none published | displacement 0, 0 non-zero commands |
| narrow_passage (2.2 m gap) | success | GOAL_RECEIVED → TRAJECTORY_PUBLISHED → TRAJECTORY_COMPLETE | 20.79 m, 32.5 s, 0.046 s | 2 mm; hull 0.610 m; circle 0.419 m; tracking ≤ 2 mm |
| narrow_too_tight (1.4 m gap) | failure | GOAL_RECEIVED → FAILED_NO_PATH | none | displacement 0, 0 commands. Refused by design: the circle needs 1.77 m |
| blocked | failure | GOAL_RECEIVED → FAILED_NO_PATH | none | displacement 0, 0 commands |

Notes:
- The circle clearance in `obstacle_straight` (0.272 m) is slightly below the 0.30 m margin. The margin is a soft
  penalty in the optimizer, and obstacle cells are quantized to 0.1 m. The hard verification limit (≥ 0) holds.
- **Not exercised:** all three failures were caught by JPS, so the `FAILED_TRAJECTORY_INFEASIBLE` branch (optimizer
  output rejected by the final check) has no scenario yet. The failure during a replan (robot moving) is also
  untested, because replanning is off (`replan_period` 5000 s).
- **Obstacles in `hull_sim`:** the hull simulator still only knows the pool walls. Actual contact with obstacles is
  evaluated after the run, geometrically, by `nav_tests.py`.
- **Regressions:** all pass after this stage: hull tests, `pool_sim.launch` (live check 61/61 + drive), and the
  stock `planner_sim` headless test.

---

## 8. Open questions

- Full mission spec beyond "start from a standby position near a corner": victim detection, pickup behaviour,
  return-to-base, time limits?
- Confirm or replace every `assumption:` in `pool_rescue/config/pool_rescue.yaml`, especially pool size, hull size,
  thruster limits and latency.
- Fidelity of water dynamics wanted (kinematic + ICR approximation vs. a 3-DOF Fossen-style surge/sway/yaw model).
- Should persons become planner obstacles (`person_clearance`) and should mode switching happen at runtime?
