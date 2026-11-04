# DDR-opt

DDR-opt is a universal Trajectory Optimization Framework for Differential Drive Robot Class.

The paper is published in T-ASE!

Please visit our project website [DDR-opt](https://zju-fast-lab.github.io/DDR-opt/).
If you find this work useful or interesting, please kindly give us a star ⭐, thanks! 😀

<p align="center">
    <img src="others/picture/head_figure_trajectory2.png" alt="Trajectory" width="70%">
</p>

## Quick Start
Compiling tests passed on ubuntu **18.04, and 20.04** with ros installed.
You can just execute the following commands one by one.
### Dependence:
``` bash
sudo apt install ros-${ROS_DISTRO}-tf2-sensor-msgs # noetic or melodic
```

OSQP and OSQP-Eigen make it easier to modify parameters and are used to solve control problems under velocity and angular velocity control.
You can download them from the following two links:
- Download [osqp-v0.6.3-src.tar.gz](https://github.com/osqp/osqp/releases/tag/v0.6.3) or click [here](https://github.com/osqp/osqp/releases/download/v0.6.3/osqp-v0.6.3-src.tar.gz), and then follow the [installation instructions](https://osqp.org/docs/get_started/sources.html)
- Download [OSQP-eigen v0.8.1](https://github.com/robotology/osqp-eigen/releases/tag/v0.8.1).
```bash
cd osqp-eigen
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ../
make
sudo make install
```

**NOTE:** We may have forgotten other dependencies 😟, sorry! If you could provide missing dependencies, we would greatly appreciate it. 

### Build
``` bash
mkdir -p DDRopt_ws/src
cd DDRopt_ws/src
git clone git@github.com:ZJU-FAST-Lab/DDR-opt.git
cd ..
catkin build
```

### IMPORTANT: For this device, run the following commends before running plan_manager launch files  ###
``` bash
cd ~/main/pr_robot/DDRopt_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash    ## Must run this command
rospack find plan_manager
```

### Run
You can run any of the following: 
``` bash
roslaunch plan_manager planner_nmpc.launch # for robots controlled by wheel speeds
roslaunch plan_manager planner_sim_unknown.launch # for planning in unknown space
roslaunch plan_manager planner_sim.launch # for robots controlled by linear and angular velocity
```

You can use `2D Nav Goal` to set goal point.  

### Robot model (branch `DDR-test-v1`): ROV V4 by default, original car with `robot:=car`
All three launch files take `robot:=rov` (default) or `robot:=car`:
``` bash
roslaunch plan_manager planner_sim.launch                 # ROV V4
roslaunch plan_manager planner_sim.launch robot:=car      # original scout_mini car, unchanged
```
- **ROV V4** (`ROV_simple_model_V4`, mesh `utils/simulator/urdf/ROV_V4.STL`).
  - `plan_manager/config/robot_rov.yaml` is loaded on top of the stock configs. It sets the limits, the collision
    circle and the mesh.
  - Collision radius 0.722 m (assembly 1.0 × 1.043 m including thrusters). Optimizer target 1.022 m, path search
    0.872 m, final rejection below 0.722 m.
  - Limits are placeholders until mass, drag and T500 thrust data exist: 1.0 / −0.3 m/s, 0.4 m/s², 0.8 rad/s.
  - DDR-opt treats the ROV as a differential-drive robot (surge + yaw). It doesn't use the ROV's sideways (sway)
    ability.
- **Environments** for the ROV, which is too large for the car's maps:
  - `planner_sim`: 30 random boxes instead of 100, default `map_seed` 4. `map_seed:=-1` gives a new random map,
    with a route in about 40% of seeds.
  - `planner_nmpc` / `planner_sim_unknown`: `obs5_rov.yaml` (`obs5` minus 2 boxes), start (−7.1, −6.1).
- Other args: `gui:=false` (no rviz/plotjuggler), `map_seed:=N`.
- Headless check: `scripts/run_headless_sim.sh --launch planner_sim|planner_nmpc|planner_sim_unknown --robot rov|car`
  (exit 0 = PASS).

## Update
25-12-02
1. Fixed the bug reported in [Issue 16](https://github.com/ZJU-FAST-Lab/DDR-opt/issues/16). Thanks to JMang0321 for the report!


25-07-25  
1. Enhanced the 'if_directly_constrain_v_omega' feature. Users can now select between: 1) Constraining linear (v) and angular (omega) velocities independently, or 2) Constraining the product (v * omega) considering wheel speed limits.
2. Reduced the number of ROS messages in the code that do not use topic remapping. 
3. Fixed the bug reported in [Issue 11](https://github.com/ZJU-FAST-Lab/DDR-opt/issues/11). Thanks to SCUTBob for the report!

## Citing
The method used in this software are described in the following paper (available on [IEEE](https://ieeexplore.ieee.org/document/10924228) and [arxiv](https://arxiv.org/abs/2409.07924v3))
```
@ARTICLE{zhang2024universaltrajectoryoptimizationframework,
  author={Zhang, Mengke and Chen, Nanhe and Wang, Hu and Qiu, Jianxiong and Han, Zhichao and Ren, Qiuyu and Xu, Chao and Gao, Fei and Cao, Yanjun},
  journal={IEEE Transactions on Automation Science and Engineering}, 
  title={Universal Trajectory Optimization Framework for Differential Drive Robot Class}, 
  year={2025},
  volume={22},
  number={},
  pages={13030-13045},
  keywords={Robots;Mobile robots;Kinematics;Trajectory optimization;Planning;Robot kinematics;Computational modeling;Dynamics;Wheels;Tracking;Motion planning;trajectory optimization;differential drive robot class;nonholonomic dynamics},
  doi={10.1109/TASE.2025.3550676}}
```
