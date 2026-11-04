#!/usr/bin/env python3
"""Headless end-to-end check for the DDR-opt simulation stack.

Sends one goal on /move_base_simple/goal, records the planned trajectory,
the controller commands and the ground-truth robot motion, then writes a
report (stdout + JSON + CSV) and exits 0 on PASS, 1 on FAIL.

Usage (normally via scripts/run_headless_sim.sh):
  rosrun-free:  python3 smoke_test.py --goal X Y YAW [--timeout S] [--out DIR]
"""
import argparse
import csv
import json
import math
import os
import sys

import rospy
from carstatemsgs.msg import CarState, Polynome
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SmokeTest:
    def __init__(self, a):
        self.a = a
        self.odom = None            # EKF odom (what planner/controller see)
        self.traj_msgs = []         # Polynome messages
        self.planned_path = []      # (x, y) from /visualizer/mincoPath (last plan)
        self.planned_all = []       # (x, y) of every published plan (replanning launches publish many)
        self.cmds = []              # (t, v, omega) from /simulation/PoseSub
        self.truth = []             # (t, x, y, yaw, v, omega) from /simulation/PosePub
        self.estop = False

        rospy.Subscriber("/ICREKF/EKF_XYTheta", Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber("/planner/traj_poly", Polynome, self.cb_traj, queue_size=10)
        rospy.Subscriber("/visualizer/mincoPath", Path, self.cb_path, queue_size=10)
        rospy.Subscriber("/simulation/PoseSub", CarState, self.cb_cmd, queue_size=100)
        rospy.Subscriber("/simulation/PosePub", CarState, self.cb_truth, queue_size=100)
        rospy.Subscriber("/planner/emergency_stop", Bool, self.cb_estop, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)

    # ---------------- callbacks ----------------
    def cb_odom(self, m):
        self.odom = m

    def cb_traj(self, m):
        self.traj_msgs.append(m)

    def cb_path(self, m):
        self.planned_path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        self.planned_all.extend(self.planned_path[::5])

    def cb_cmd(self, m):
        self.cmds.append((rospy.get_time(), m.v, m.omega))

    def cb_truth(self, m):
        self.truth.append((rospy.get_time(), m.x, m.y, m.yaw, m.v, m.omega))

    def cb_estop(self, m):
        if m.data:
            self.estop = True

    # ---------------- run ----------------
    def wait_for(self, cond, timeout, what):
        t0 = rospy.get_time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and not cond():
            if rospy.get_time() - t0 > timeout:
                rospy.logerr("timeout waiting for %s", what)
                return False
            r.sleep()
        return True

    def send_goal(self):
        x, y, yaw = self.a.goal
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y = x, y
        q = quaternion_from_euler(0, 0, yaw)
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = q
        self.goal_pub.publish(g)
        rospy.loginfo("goal sent: x=%.2f y=%.2f yaw=%.2f", x, y, yaw)

    def run(self):
        if not self.wait_for(lambda: self.odom is not None, 30, "EKF odometry"):
            return self.finish("no odometry")
        # Let the laser -> SDF map -> ESDF pipeline fill the map before planning.
        rospy.sleep(self.a.map_settle)
        self.send_goal()
        if not self.wait_for(lambda: self.traj_msgs or self.estop, 20, "trajectory"):
            return self.finish("no trajectory published")
        if self.estop and not self.traj_msgs:
            return self.finish("emergency stop before planning")

        first = self.traj_msgs[0]
        duration = sum(first.t_pts)
        t_end = first.traj_start_time.to_sec() + duration
        rospy.loginfo("trajectory: %d pieces, %.2f s; waiting for execution", len(first.t_pts), duration)

        # Wait for the trajectory to finish (+ settle) or the timeout.
        deadline = rospy.get_time() + self.a.timeout
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self.estop:
                break
            last_t = max(t_end, self.traj_msgs[-1].traj_start_time.to_sec() + sum(self.traj_msgs[-1].t_pts))
            if rospy.get_time() > last_t + self.a.settle:
                break
            r.sleep()
        return self.finish(None)

    # ---------------- report ----------------
    def finish(self, early_fail):
        a = self.a
        rep = {"goal": a.goal, "early_failure": early_fail, "emergency_stop": self.estop,
               "num_trajectories": len(self.traj_msgs)}
        checks = {}

        if self.traj_msgs:
            m = self.traj_msgs[0]
            rep["traj"] = {
                "pieces": len(m.t_pts),
                "duration_s": sum(m.t_pts),
                "start_xyyaw": [m.start_position.x, m.start_position.y, m.start_position.z],
                "final_yaw_rad": m.tail_p.x,
                "final_arclength_m": m.tail_p.y,
            }
        if self.planned_path:
            px, py = self.planned_path[-1]
            rep["planned_path"] = {"points": len(self.planned_path), "end_xy": [px, py],
                                   "end_to_goal_m": math.hypot(px - a.goal[0], py - a.goal[1])}

        if self.cmds:
            vs = [c[1] for c in self.cmds]
            ws = [c[2] for c in self.cmds]
            rep["cmd"] = {"count": len(self.cmds), "v_min": min(vs), "v_max": max(vs),
                          "omega_min": min(ws), "omega_max": max(ws)}
            checks["cmd_v_within_limits"] = a.min_vel - 1e-3 <= min(vs) and max(vs) <= a.max_vel + 1e-3
            checks["cmd_omega_within_limits"] = max(abs(w) for w in ws) <= a.max_omega + 1e-3

        if self.truth:
            t0 = self.truth[0]
            tN = self.truth[-1]
            dist = sum(math.hypot(b[1] - c[1], b[2] - c[2]) for b, c in zip(self.truth[1:], self.truth[:-1]))
            pos_err = math.hypot(tN[1] - a.goal[0], tN[2] - a.goal[1])
            yaw_err = abs(wrap(tN[3] - a.goal[2]))
            rep["motion"] = {"start_xyyaw": list(t0[1:4]), "final_xyyaw": list(tN[1:4]),
                             "distance_travelled_m": dist, "final_v": tN[4], "final_omega": tN[5],
                             "final_pos_err_m": pos_err, "final_yaw_err_rad": yaw_err}
            checks["robot_moved"] = dist > 0.5
            checks["reached_goal_pos"] = pos_err < a.pos_tol
            checks["stopped"] = abs(tN[4]) < 0.05 and abs(tN[5]) < 0.05

            if self.planned_path:
                # cross-track error: distance from each actual sample to the nearest point of ANY published plan
                # (with periodic replanning the robot follows the plan that was active at that time)
                import numpy as np   # vectorised: replanning launches publish 100+ plans
                ref = np.array(self.planned_all or self.planned_path)
                pts = np.array([(o[1], o[2]) for o in self.truth[::5]])
                errs = []
                for k in range(0, len(pts), 200):
                    d = np.linalg.norm(pts[k:k + 200, None, :] - ref[None, :, :], axis=2)
                    errs.extend(d.min(axis=1).tolist())
                rep["tracking"] = {"max_err_m": max(errs), "mean_err_m": sum(errs) / len(errs),
                                   "reference_points": int(len(ref))}
                checks["tracking_err_ok"] = max(errs) < a.track_tol

        checks["trajectory_published"] = bool(self.traj_msgs)
        checks["no_emergency_stop"] = not self.estop
        checks["no_early_failure"] = early_fail is None
        rep["checks"] = checks
        rep["result"] = "PASS" if all(checks.values()) else "FAIL"

        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, "report.json"), "w") as f:
            json.dump(rep, f, indent=2)
        with open(os.path.join(a.out, "truth.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw", "v", "omega"])
            w.writerows(self.truth)
        with open(os.path.join(a.out, "cmd.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "omega"])
            w.writerows(self.cmds)
        with open(os.path.join(a.out, "planned_path.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y"])
            w.writerows(self.planned_path)

        print(json.dumps(rep, indent=2))
        print("RESULT:", rep["result"])
        return rep["result"] == "PASS"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--goal", nargs=3, type=float, default=[3.7, 0.3, 3.14], metavar=("X", "Y", "YAW"))
    p.add_argument("--timeout", type=float, default=60.0, help="max seconds to wait for execution")
    p.add_argument("--map-settle", type=float, default=3.0, help="seconds to let the map build before the goal")
    p.add_argument("--settle", type=float, default=2.0, help="seconds after trajectory end before judging")
    p.add_argument("--pos-tol", type=float, default=0.3)
    p.add_argument("--track-tol", type=float, default=0.3)
    p.add_argument("--max-vel", type=float, default=3.0)
    p.add_argument("--min-vel", type=float, default=-3.0)
    p.add_argument("--max-omega", type=float, default=3.0)
    p.add_argument("--out", default="smoke_out")
    a = p.parse_args(rospy.myargv()[1:])
    rospy.init_node("smoke_test", anonymous=True)
    ok = SmokeTest(a).run()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
