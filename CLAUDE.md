# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

DDR-opt is a ROS1 (catkin) trajectory optimization framework for differential-drive robots (T-ASE 2025, Zhang et al.). This repo is the `src/` content of the catkin workspace at `~/main/pr_robot/DDRopt_ws`. On this machine it runs on ROS Noetic.

## Build & Run

Every build/run must be done from the workspace root with ROS and the workspace sourced (launch files fail with "package not found" otherwise):

```bash
cd ~/main/pr_robot/DDRopt_ws
source /opt/ros/noetic/setup.bash
catkin build                      # whole workspace (catkin_tools, devel layout = linked)
catkin build <pkg>                # single package, e.g. catkin build back_end
source devel/setup.bash           # required after building, before roslaunch
rospack find plan_manager         # sanity check
```

Launch the full simulation stack (planner + simulator + controller + EKF + laser sim + rviz + plotjuggler):

```bash
roslaunch plan_manager planner_sim.launch          # robot controlled by linear/angular velocity (OSQP MPC)
roslaunch plan_manager planner_nmpc.launch         # robot controlled by wheel speeds (ACADO NMPC)
roslaunch plan_manager planner_sim_unknown.launch  # planning in unknown space (uses *_unknown.yaml configs)
```

Set goals in rviz with `2D Nav Goal` (`/move_base_simple/goal`). There are no unit tests; verification is done by running the simulation.

External deps not managed by catkin: OSQP v0.6.3 and OSQP-Eigen v0.8.1 installed to `/usr/local` (needed by `mpc_controller` and `nmpc_controller`), `ros-noetic-tf2-sensor-msgs`, and `plotjuggler`.

## Architecture

Pipeline (topics as wired in the launch files):

```
global_map_node (simulator) ──> laser_sim_node ──/laser_simulator/local_pointcloud──┐
simulator ──pose──> icrekf ──/ICREKF/EKF_XYTheta (odom)──> global_planning (plan_manager) <── /move_base_simple/goal
                                                     │
                     /planner/traj_poly (carstatemsgs/Polynome) + /planner/emergency_stop
                                                     ▼
                         mpc (mpc_controller) or nmpc (nmpc_controller)
                                                     │
                 /simulation/PoseSub (CarState v/ω)  or  wheel_cmd (CarControl)  ──> simulator
```

- **plan_manager** — the `global_planning` node. Nearly all logic lives in the header `include/plan_manager/plan_manager.hpp` (`PlanManager`); `src/global_planning.cpp` is just `main`. A 1 ms timer (`MainThread`) runs the state machine: collision check → replan decision (`replan_time`, `max_replan_time`) → `findJPSRoad()` (front end) → `msplanner_->minco_plan(...)` (back end) → publish `Polynome` via `MPCPathPub`. Owns the `SDFmap`, `JPSPlanner`, `MSPlanner`, and `Visualizer`.
- **front_end** — JPS grid search (`jps_planner/`) producing a kinodynamic path / `FlatTrajData` (`traj_representation.h`) used to initialize optimization.
- **back_end** — `MSPlanner` in `optimizer.{h,cpp}` (~1850 lines): MINCO-based trajectory optimization in flat (yaw, arc-length) space solved with L-BFGS; two stages (path pre-processing then full optimization) with penalty weights from `config/global_planning*.yaml`. `include/gcopter/` is vendored GCOPTER math (minco, lbfgs, sdlp, root_finder, trajectory). Supports non-standard diff drive via ICR parameters (`ICR_yr/yl/xv`, `if_standard_diff`) and `if_directly_constrain_v_omega`.
- **mpc_controller** — linear MPC (OSQP-Eigen) tracking the `Polynome` trajectory; outputs v/ω.
- **nmpc_controller** — ACADO-generated NMPC (code in `UAV_CAR_model/build/quadrotor_mpc_codegen`, qpOASES vendored in `externals/`); outputs wheel speeds and consumes `EKF_ICR`. Regenerating the solver requires ACADO (see `others/UAV_CAR_model/README.md`); normally don't touch the generated code.
- **icrekf** — EKF estimating pose plus ICR parameters (yr, yl, xv) for skid-steer/tracked robots.
- **utils/** — `carstatemsgs` (messages: `Polynome`, `CarState`, `CarControl`, ...), `plan_env` (`SDFmap` + raycasting; map/obstacle yaml files), `simulator` (robot kinematic sim + `global_map_node` which generates maps: method 1 yaml, 2 random boxes, 3 png), `laser_simulator`.

## Configuration conventions

- Parameters are split across packages and loaded by the launch files: `front_end/config/jps<suffix>.yaml`, `plan_manager/config/car<suffix>.yaml` (robot limits, shared by planner/simulator/controllers), `back_end/config/global_planning<suffix>.yaml`, `{mpc,nmpc}_controller/config/mpc<suffix>.yaml`. `<suffix>` comes from the launch arg `velocity_suffix` (`3ms`) — add a new config set by creating files with a new suffix and changing that arg. `*_unknown.yaml` variants are used by `planner_sim_unknown.launch`.
- Scenario settings (start/goal, `if_fix_final_`, map method, ICR values, sensor range/FOV, `replan_time`) are launch-file args/params, not yaml.
- Nodes use private namespaces (`~odom`, `~traj`, `~cmd`) remapped in launch files, but some topics are hard-coded absolute (`/planner/emergency_stop`, `/move_base_simple/goal`, `/simulation/PoseSub`, `/mpc/*` in mpc_controller).
- Some launch/source comments in this checkout are in Chinese (local annotations, e.g. `planner_sim.launch`).
