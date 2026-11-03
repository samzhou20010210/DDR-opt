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
| 1 | FAIL | JPS: no path to goal (random boxes only keep clear of the start) → **`global_planning` segfaults** |
| 2 | PASS | 19 pieces, 7.62 s, 11.58 m; plan 24 ms (JPS 0.4 ms, pre-process 10 ms); min clearance 0.31 m; cmd v ∈ [−0.30, 2.78], ω ∈ [−0.88, 2.57]; final err 1.6 cm / 0.003 rad; max tracking err 2.6 cm |
| 3 | FAIL | same as seed 1 |
| 4 | PASS | |

Repeat runs of seed 2 give bit-identical plans; final pose varies by ~1 cm because the simulator adds velocity
noise (`if_add_noise: true`).

**Known bug (not fixed):** `PlanManager::findJPSRoad()` ignores the return value of `JPSPlanner::plan()`; when no
path exists it continues on an empty path and the node crashes (`exit code -11`). Needs a guard before the
pool-rescue mission logic relies on replanning.

---

## 4. Change log

| Date | Change | Files |
|---|---|---|
| 2026-11-03 | Created this log: baseline analysis, interfaces, assumptions for the pool-rescue extension. No code changed. | `DEVLOG.md` |
| 2026-11-03 | Headless workflow: `gui` (default true) and `map_seed` (default −1) args in `planner_sim.launch` — defaults keep stock behaviour; headless runner + automated goal/trajectory/control/motion check; baseline results recorded. | `plan_manager/launch/planner_sim.launch`, `scripts/run_headless_sim.sh`, `scripts/smoke_test.py`, `DEVLOG.md` |

---

## 5. Open questions

- Full mission spec (text cut off after "starting from a standby position near a corne…"): victim detection, pickup
  behaviour, return-to-base, time limits?
- Pool geometry (dimensions, shape, lane ropes/ladders as obstacles).
- Robot parameters: hull size, mass, thruster max thrust, track width between thrusters, drag coefficients.
- Fidelity of water dynamics wanted (kinematic + ICR approximation vs. a 3-DOF Fossen-style surge/sway/yaw model).
- Controller: v/ω MPC or thruster-level NMPC.
