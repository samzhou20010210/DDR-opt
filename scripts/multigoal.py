#!/usr/bin/env python3
"""Multi-goal check against a running plan_manager launch, for robot:=rov_holo (holonomic) and rov / car (diff drive).

Sends goals one after another on /move_base_simple/goal (like clicking 2D Nav Goal). For each goal it records:
- outcome, number of plans, goal snapping;
- executed path length, drive time, total time, average speed;
- minimum clearance: distance from the robot centre to the nearest occupied cell of the ground-truth map
  (/global_map/gridmap; subtract the collision radius, 0.722 m for the ROV, to get the gap to the hull);
- first-plan duration and length;
- tracking: cross-track distance to the planned paths (both models), and for rov_holo also the time-indexed pose error
  against the NMPC reference (/holo_nmpc/ref_pose);
- final error relative to the (snapped) goal.

For rov_holo only:
- planner-side endpoint error and peak / limit ratio, parsed from the planner log;
- velocity peaks, sway use and commands outside the limits;
- NMPC solve statistics;
- thruster allocation: saturation, thrusters at a limit, and residual heave / roll / pitch.

  python3 multigoal.py --out DIR [--robot rov_holo|rov|car] [--goals x y yaw ...] [--timeout S]
                       [--gazebo]   (rov_holo in Gazebo: rov_gazebo_bridge topics, pool walls, depth hold)
Exit 0 = every goal reached (and, for rov_holo, no command outside the limits). Writes DIR/multigoal.json.
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, Float64MultiArray
from carstatemsgs.msg import Polynome, CarState
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# user's 6 clicked goals + 8 random goals across the map (ROV map: 20 boxes, seed 10)
DEFAULT_GOALS = [5.77, -7.78, 1.58, 4.12, 8.42, -0.28, 7.57, 5.88, -1.95, 4.87, 8.01, -0.52, 8.32, 6.72, -1.69,
                 7.45, -2.35, 2.0, 2.38, 7.55, 0.0, 5.24, -5.22, 0.0, -3.8, 7.1, 0.0, -9.4, 6.1, 0.0, 5.64, -0.61, 0.0,
                 -3.74, -4.21, 0.0, -4.66, -1.04, 0.0, 0.09, 1.02, 0.0]
# Gazebo pool (rov_gazebo_bridge, frame = pool corner, pool x in [0, 25], y in [0, 50]): 14 goals over the pool
GAZEBO_GOALS = [12.0, 15.0, 1.57, 20.0, 40.0, 0.0, 3.0, 45.0, 3.14, 22.0, 5.0, -1.57, 12.5, 25.0, 0.0, 2.0, 2.0, 0.785,
                23.0, 48.0, 1.57, 8.0, 30.0, -1.0, 18.0, 12.0, 2.5, 5.0, 20.0, 0.0, 20.0, 25.0, 3.14, 1.5, 40.0, -1.57,
                15.0, 47.0, 0.0, 5.0, 5.0, 0.0]
NMPC_BUDGET_MS = 20.0   # 50 Hz control rate


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def stats(xs, nd=3):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    a = np.array(xs, dtype=float)
    return {"mean": round(float(a.mean()), nd), "p95": round(float(np.percentile(a, 95)), nd),
            "max": round(float(a.max()), nd), "n": int(a.size)}


class Recorder:
    def __init__(self, holo, topics):
        self.holo = holo
        self.depth = []       # (t, depth)
        self.pose = []        # (t, x, y, yaw)  50 Hz ground truth
        self.odom = []        # (t, vx, vy, w)  holo: body twist
        self.carstate = []    # (t, v, w)       diff
        self.cmds = []        # (t, vx, vy, w)
        self.trajs = []       # (t_recv, t_start, duration, tail (x, y, yaw), path at that time)
        self.paths = []       # (t, [(x, y)])
        self.sat, self.thrust, self.wach, self.nmpc, self.ref = [], [], [], [], []
        self.endpoint, self.ratio, self.attempts = [], [], []
        self.drops, self.snaps, self.estops = [], [], []
        self.obst = None
        rospy.Subscriber(topics["pose"], PoseStamped, self.cb_pose, queue_size=500)
        if topics.get("depth"):
            rospy.Subscriber(topics["depth"], Float64, lambda m: self.depth.append((rospy.get_time(), m.data)), queue_size=500)
        rospy.Subscriber("/planner/traj_poly", Polynome, self.cb_traj, queue_size=20)
        rospy.Subscriber("/visualizer/mincoPath", Path, self.cb_path, queue_size=20)
        rospy.Subscriber("/planner/emergency_stop", Bool, lambda m: m.data and self.estops.append(rospy.get_time()))
        rospy.Subscriber("/rosout_agg", Log, self.cb_log, queue_size=500)
        rospy.Subscriber(topics["map"], PointCloud2, self.cb_map, queue_size=1)
        if holo:
            rospy.Subscriber(topics["odom"], Odometry, self.cb_odom, queue_size=500)
            rospy.Subscriber("/simulation/HoloCmd", Twist, self.cb_cmd, queue_size=500)
            rospy.Subscriber("/thruster_allocation/saturated", Bool, self.cb_sat, queue_size=500)
            rospy.Subscriber("/thruster_allocation/thrust", Float64MultiArray, self.cb_thrust, queue_size=500)
            rospy.Subscriber("/thruster_allocation/wrench_achieved", WrenchStamped, self.cb_wach, queue_size=500)
            rospy.Subscriber("/holo_nmpc/stats", Float64MultiArray, self.cb_nmpc, queue_size=500)
            rospy.Subscriber("/holo_nmpc/ref_pose", PoseStamped, self.cb_ref, queue_size=500)
        else:
            rospy.Subscriber("/simulation/PosePub", CarState, self.cb_car, queue_size=500)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    def cb_pose(self, m):
        self.pose.append((rospy.get_time(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))

    def cb_odom(self, m):
        t = m.twist.twist
        self.odom.append((rospy.get_time(), t.linear.x, t.linear.y, t.angular.z))

    def cb_car(self, m):
        self.carstate.append((rospy.get_time(), m.v, m.omega))

    def cb_cmd(self, m):
        self.cmds.append((rospy.get_time(), m.linear.x, m.linear.y, m.angular.z))

    def cb_traj(self, m):
        self.trajs.append([rospy.get_time(), m.traj_start_time.to_sec(), sum(m.t_pts),
                           (m.tail_p.x, m.tail_p.y, m.tail_p.z), []])

    def cb_path(self, m):
        self.paths.append((rospy.get_time(), [(p.pose.position.x, p.pose.position.y) for p in m.poses]))

    def path_for(self, tr):
        """planned path published together with trajectory tr (the topics arrive in either order)"""
        c = [p for p in self.paths if abs(p[0] - tr[0]) < 0.5]
        return min(c, key=lambda p: abs(p[0] - tr[0]))[1] if c else []

    def cb_sat(self, m):
        self.sat.append((rospy.get_time(), m.data))

    def cb_thrust(self, m):
        self.thrust.append((rospy.get_time(), list(m.data)))

    def cb_wach(self, m):
        w = m.wrench
        self.wach.append((rospy.get_time(), abs(w.force.z), abs(w.torque.x), abs(w.torque.y)))

    def cb_nmpc(self, m):
        self.nmpc.append((rospy.get_time(),) + tuple(m.data))

    def cb_ref(self, m):
        self.ref.append((m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))

    def cb_map(self, m):
        if self.obst is None:
            pts = np.array([(p[0], p[1]) for p in pc2.read_points(m, field_names=("x", "y"), skip_nans=True)])
            if len(pts):
                self.obst = pts

    def cb_log(self, m):
        t = rospy.get_time()
        if "Goal dropped" in m.msg:
            self.drops.append(t)
        mm = re.search(r"moved ([0-9.]+) m to \(([-0-9.]+), ([-0-9.]+)\)", m.msg)
        if mm:
            self.snaps.append((t, float(mm.group(1)), float(mm.group(2)), float(mm.group(3))))
        mm = re.search(r"endpoint error pos ([-0-9.e+]+) m yaw ([-0-9.e+]+) rad vel ([-0-9.e+]+)", m.msg)
        if mm:
            self.endpoint.append((t, float(mm.group(1)), float(mm.group(2)), float(mm.group(3))))
        mm = re.search(r"max_ratio ([-0-9.e+]+)", m.msg)
        if mm:
            al = re.search(r"alloc ([-0-9.e+]+)", m.msg)
            self.ratio.append((t, float(mm.group(1)), float(al.group(1)) if al else None))
        if re.search(r"\[holo\] attempt \d+:", m.msg):
            self.attempts.append(t)


def clearance(poses, obst):
    if obst is None or not poses:
        return None
    P = np.array([(p[1], p[2]) for p in poses[::2]])
    best = np.inf
    for k in range(0, len(P), 200):
        d = np.sqrt(((P[k:k + 200, None, :] - obst[None, :, :]) ** 2).sum(-1)).min()
        best = min(best, d)
    return float(best)


def rate_peaks(od, half=0.1):
    """peak |dvx/dt|, |dvy/dt|, |domega/dt| from the body twist (central difference over +-half s, 0.2 s window)"""
    if len(od) < 3:
        return [None] * 3
    t = np.array([o[0] for o in od])
    V = np.array([o[1:4] for o in od])
    out = np.zeros(3)
    j0 = j1 = 0
    for i in range(len(t)):
        while t[j0] < t[i] - half:
            j0 += 1
        while j1 + 1 < len(t) and t[j1 + 1] <= t[i] + half:
            j1 += 1
        if t[j1] - t[j0] > 1.5 * half:
            out = np.maximum(out, np.abs((V[j1] - V[j0]) / (t[j1] - t[j0])))
    return [round(float(x), 3) for x in out]


def jitter(th):
    """max over thrusters of the RMS change between consecutive thrust commands (N)"""
    if len(th) < 3:
        return None
    F = np.array([f[1] for f in th])
    return round(float(np.sqrt((np.diff(F, axis=0) ** 2).mean(axis=0)).max()), 3)


def raw_acc(od):
    """max body acceleration from 1-step differences (detects high-frequency oscillation)"""
    if len(od) < 3:
        return None
    t = np.array([o[0] for o in od])
    V = np.array([o[1:3] for o in od])
    dt = np.maximum(np.diff(t), 1e-3)
    return round(float((np.linalg.norm(np.diff(V, axis=0), axis=1) / dt).max()), 3)


def path_len(pts):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    p.add_argument("--robot", default="rov_holo")
    p.add_argument("--goals", nargs="+", type=float, default=None)
    p.add_argument("--timeout", type=float, default=150.0)
    p.add_argument("--pos-tol", type=float, default=0.3)
    p.add_argument("--yaw-tol", type=float, default=0.3)
    p.add_argument("--gazebo", action="store_true")
    a = p.parse_args(rospy.myargv()[1:])
    holo = a.robot == "rov_holo"
    if a.goals is None:
        a.goals = GAZEBO_GOALS if a.gazebo else DEFAULT_GOALS
    os.makedirs(a.out, exist_ok=True)
    if a.gazebo:   # rospy picks wall or sim time at init_node: wait until Gazebo's launch has set /use_sim_time
        import rosgraph
        def _sim_time_set():
            try:
                return rosgraph.Master("/multigoal").hasParam("/use_sim_time")
            except Exception:   # master not up yet
                return False
        while not _sim_time_set():
            __import__("time").sleep(0.2)
    rospy.init_node("multigoal", anonymous=True)
    key = "/holo_nmpc/holo/max_vy" if holo else "/global_planning"
    while not rospy.is_shutdown() and not rospy.has_param(key):
        rospy.sleep(0.2)   # wait for the launch to load its parameters
    lim = {k: rospy.get_param("/holo_nmpc/holo/" + k) for k in ("max_vx", "min_vx", "max_vy", "max_omega")} if holo else {}
    vert = []
    if holo:
        vert = [i for i, z in enumerate(rospy.get_param("/thruster_allocation/thrusters/axis_z")) if abs(z) > 0.5]
        fmax, fmin = rospy.get_param("/thruster_allocation/thrusters/f_max"), rospy.get_param("/thruster_allocation/thrusters/f_min")
    topics = ({"pose": "/gazebo_bridge/pose", "odom": "/gazebo_bridge/odom", "map": "/pool_obstacles/cloud",
               "depth": "/gazebo_bridge/depth"} if a.gazebo else
              {"pose": "/simulation/PosePosePub", "odom": "/simulation/PoseOdomPub", "map": "/global_map/gridmap"})
    depth_target = rospy.get_param("/gazebo_bridge/depth/target", None) if a.gazebo else None
    R = Recorder(holo, topics)
    while not R.pose and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.sleep(4.0)  # map build

    hold = None
    if a.gazebo:   # station keeping before the first goal: oscillation check (added-mass / controller stability)
        t0h = rospy.get_time()
        rospy.sleep(10.0)
        w = lambda L: [s for s in L if s[0] >= t0h]
        od, th, dp = w(R.odom), w(R.thrust), [d[1] for d in w(R.depth)]
        sp = [math.hypot(o[1], o[2]) for o in od]
        hold = {"speed_std": round(float(np.std(sp)), 4) if sp else None, "speed_max": round(max(sp), 4) if sp else None,
                "thrust_step_rms_N": jitter(th), "raw_acc_max": raw_acc(od),
                "depth_err_max_m": round(max(abs(d - depth_target) for d in dp), 3) if dp else None}
        print("[multigoal] hold 10 s: %s" % json.dumps(hold), flush=True)

    goals = [tuple(a.goals[i:i + 3]) for i in range(0, len(a.goals) - 2, 3)]
    results = []
    for gx, gy, gyaw in goals:
        n_tr, n_dr, n_sn, n_es, t0 = len(R.trajs), len(R.drops), len(R.snaps), len(R.estops), rospy.get_time()
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y = gx, gy
        qx, qy, qz, qw = quaternion_from_euler(0, 0, gyaw)
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = qx, qy, qz, qw
        R.goal_pub.publish(g)
        outcome, t_end = "TIMEOUT", None
        while rospy.get_time() - t0 < a.timeout and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if len(R.trajs) == n_tr and len(R.drops) > n_dr:
                outcome = "NO_PATH"
                break
            if len(R.estops) > n_es:
                outcome = "ESTOP"
                break
            if len(R.trajs) > n_tr:
                _, ts, dur, _, _ = R.trajs[-1]
                recent = [s for s in R.pose if s[0] > rospy.get_time() - 0.5]
                moved = path_len([(s[1], s[2]) for s in recent]) + sum(abs(wrap(b[3] - c[3])) for c, b in zip(recent[:-1], recent[1:]))
                if rospy.get_time() > ts + dur + 1.0 and moved < 0.01:
                    outcome, t_end = "STOPPED", rospy.get_time()
                    break
        rospy.sleep(0.3)
        t1 = rospy.get_time()
        win = lambda L: [s for s in L if t0 <= s[0] <= t1]
        seg = win(R.pose)
        trajs = R.trajs[n_tr:]
        for tr in trajs:
            tr[4] = R.path_for(tr)
        snapped = R.snaps[n_sn] if len(R.snaps) > n_sn else None
        tx, ty = (snapped[2], snapped[3]) if snapped else (gx, gy)
        end = seg[-1]
        err = math.hypot(end[1] - tx, end[2] - ty)
        yerr = abs(wrap(end[3] - gyaw))
        dist = path_len([(s[1], s[2]) for s in seg])
        moving = [(b[0], math.hypot(b[1] - c[1], b[2] - c[2]) / max(b[0] - c[0], 1e-3), abs(wrap(b[3] - c[3])) / max(b[0] - c[0], 1e-3))
                  for c, b in zip(seg[:-1], seg[1:])]
        moving = [m for m in moving if m[1] > 0.02 or m[2] > 0.02]
        t_move = (moving[-1][0] - moving[0][0]) if len(moving) > 1 else 0.0
        pts = np.array([q for tr in trajs for q in tr[4]]) if trajs and any(tr[4] for tr in trajs) else None
        track = None
        if pts is not None and len(pts):
            P = np.array([(s[1], s[2]) for s in seg[::5]])
            track = np.sqrt(((P[:, None, :] - pts[None, ::2, :]) ** 2).sum(-1)).min(axis=1)
        reached = outcome == "STOPPED" and err < a.pos_tol and yerr < a.yaw_tol
        r = {"goal": [gx, gy, gyaw], "outcome": "REACHED" if reached else outcome, "plans": len(trajs),
             "snapped_m": round(snapped[1], 2) if snapped else 0.0,
             "distance_m": round(dist, 2), "drive_time_s": round(t_move, 1),
             "total_time_s": round((t_end or t1) - t0, 1),
             "avg_speed": round(dist / t_move, 2) if t_move > 0 else 0.0,
             "first_plan_duration_s": round(trajs[0][2], 2) if trajs else None,
             "first_plan_length_m": round(path_len(trajs[0][4]), 2) if trajs and trajs[0][4] else None,
             "min_clearance_m": round(clearance(seg, R.obst), 3) if seg else None,
             "final_err_m": round(err, 3), "final_yaw_err_rad": round(yerr, 3),
             "cross_track_max_m": round(float(track.max()), 3) if track is not None else None,
             "cross_track_mean_m": round(float(track.mean()), 3) if track is not None else None}
        if holo:
            od = win(R.odom)
            cmd = win(R.cmds)
            mv = [o for o in od if math.hypot(o[1], o[2]) > 0.02 or abs(o[3]) > 0.02]
            over = [c for c in cmd if c[1] > lim["max_vx"] + 1e-3 or c[1] < lim["min_vx"] - 1e-3 or
                    abs(c[2]) > lim["max_vy"] + 1e-3 or abs(c[3]) > lim["max_omega"] + 1e-3]
            # time-indexed tracking: reference pose vs ground-truth pose at the same time
            ref = win(R.ref)
            te, ye = [], []
            if ref and seg:
                ts = np.array([s[0] for s in seg])
                for q in ref[::5]:
                    k = int(np.clip(np.searchsorted(ts, q[0]), 0, len(seg) - 1))
                    s = seg[k]
                    te.append(math.hypot(s[1] - q[1], s[2] - q[2]))
                    ye.append(abs(wrap(s[3] - q[3])))
            last = trajs[-1] if trajs else None
            ctrl_end = math.hypot(end[1] - last[3][0], end[2] - last[3][1]) if last else None
            ep = win(R.endpoint)
            ra = win(R.ratio)
            th = win(R.thrust)
            hor = [i for i in range(len(th[0][1]))] if th else []
            hor = [i for i in hor if i not in vert]
            atlim = lambda f, idx: any(f[i] >= fmax - 0.5 or f[i] <= fmin + 0.5 for i in idx)
            wa = win(R.wach)
            nm = win(R.nmpc)
            sat = win(R.sat)
            r.update({
                "planner_endpoint_pos_err_m": max((e[1] for e in ep), default=None),
                "planner_endpoint_yaw_err_rad": max((e[2] for e in ep), default=None),
                "planner_endpoint_vel": max((e[3] for e in ep), default=None),
                "controller_end_err_m": round(ctrl_end, 4) if ctrl_end is not None else None,
                "track_time_err": stats(te), "track_yaw_err": stats(ye),
                "planned_peak_ratio_max": round(max((x[1] for x in ra), default=float("nan")), 4),
                "planned_alloc_max": max((x[2] for x in ra if x[2] is not None), default=None),
                "limit_retries": len(win([(t,) for t in R.attempts])),
                "peak_vx": round(max((o[1] for o in od), default=0), 3),
                "peak_abs_vy": round(max((abs(o[2]) for o in od), default=0), 3),
                "peak_abs_w": round(max((abs(o[3]) for o in od), default=0), 3),
                "peak_rates_dvx_dvy_dw": rate_peaks(od),
                "thrust_step_rms_N": jitter(th),
                "raw_acc_max": raw_acc(od),
                "sway_time_frac": round(sum(1 for o in mv if abs(o[2]) > 0.05) / max(len(mv), 1), 2),
                "cmd_outside_limits": len(over),
                "alloc_saturated_frac": round(sum(1 for s in sat if s[1]) / max(len(sat), 1), 4),
                "alloc_n": len(sat),
                "horizontal_at_limit_frac": round(sum(1 for f in th if atlim(f[1], hor)) / max(len(th), 1), 4),
                "vertical_at_limit_frac": round(sum(1 for f in th if atlim(f[1], vert)) / max(len(th), 1), 4),
                "peak_horizontal_N": round(max((abs(f[1][i]) for f in th for i in hor), default=0), 2),
                "peak_vertical_N": round(max((abs(f[1][i]) for f in th for i in vert), default=0), 2),
                "max_abs_Z_K_M": [round(max((w[k] for w in wa), default=0), 5) for k in (1, 2, 3)],
                "nmpc": {"cycle_ms": stats([n[1] for n in nm]), "osqp_solve_ms": stats([n[2] for n in nm]),
                         "osqp_iter": stats([n[3] for n in nm], 1), "failed": sum(1 for n in nm if n[4] < 0.5),
                         "period_ms": stats([n[5] for n in nm if n[5] > 0]),
                         "over_budget": sum(1 for n in nm if n[1] > NMPC_BUDGET_MS), "cycles": len(nm)}})
        if a.gazebo:
            dp = [d[1] for d in win(R.depth)]
            r["depth_err_max_m"] = round(max(abs(d - depth_target) for d in dp), 3) if dp else None
            r["depth_err_mean_m"] = round(float(np.mean([abs(d - depth_target) for d in dp])), 3) if dp else None
        if not holo:
            cs = win(R.carstate)
            r.update({"peak_v": round(max((c[1] for c in cs), default=0), 3),
                      "peak_abs_w": round(max((abs(c[2]) for c in cs), default=0), 3)})
        results.append(r)
        print(json.dumps(r), flush=True)
    ok = all(r["outcome"] == "REACHED" and r.get("cmd_outside_limits", 0) == 0 for r in results)
    summary = {"robot": a.robot, "limits_placeholder": lim, "hold": hold, "reached": sum(r["outcome"] == "REACHED" for r in results),
               "goals": len(results), "pass": ok, "results": results}
    if holo:
        nm = [n for n in R.nmpc]
        summary["nmpc_all"] = {"cycle_ms": stats([n[1] for n in nm]), "osqp_solve_ms": stats([n[2] for n in nm]),
                               "osqp_iter": stats([n[3] for n in nm], 1), "failed": sum(1 for n in nm if n[4] < 0.5),
                               "period_ms": stats([n[5] for n in nm if n[5] > 0]),
                               "over_budget": sum(1 for n in nm if n[1] > NMPC_BUDGET_MS), "cycles": len(nm)}
    json.dump(summary, open(os.path.join(a.out, "multigoal.json"), "w"), indent=1)
    print("[multigoal] %s: %d/%d goals reached -> %s" % (a.robot, summary["reached"], len(results), "PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
