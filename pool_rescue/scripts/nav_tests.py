#!/usr/bin/env python3
"""Single-goal navigation test against a running pool_nav.launch (scenario taken from /pool/scenario).

Sends the scenario goal once, records planner status, published trajectories, the planned x-y path, motion
commands and hull_sim ground truth, then judges the run against the scenario's `expect`:
  success : TRAJECTORY_PUBLISHED -> TRAJECTORY_COMPLETE, goal reached, the actual hull rectangle and the planned
            collision circle never touch an obstacle or a wall.
  failure : a FAILED_* status, no trajectory published, no non-zero motion command, vessel did not move.
Writes planned.csv, actual.csv, cmd.csv, status.csv, report.json and <scenario>.png into --out; exit 0 = PASS.
"""
import argparse
import json
import math
import os
import sys

import rospy
from carstatemsgs.msg import Polynome
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

TERMINAL = ("TRAJECTORY_COMPLETE", "FAILED_NO_PATH", "FAILED_TRAJECTORY_INFEASIBLE", "EMERGENCY_STOP_COLLISION",
            "GOAL_REJECTED_BUSY")


def P(path):
    return rospy.get_param("/pool/" + path.replace(".", "/") + "/value")


# ------------------------------------------------------------------ geometry
def box_poly(cx, cy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * bx - s * by, cy + s * bx + c * by)
            for bx, by in ((sx / 2, sy / 2), (sx / 2, -sy / 2), (-sx / 2, -sy / 2), (-sx / 2, sy / 2))]


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def inside(p, poly):
    sgn = None
    for a, b in zip(poly, poly[1:] + poly[:1]):
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cr) < 1e-12:
            continue
        if sgn is None:
            sgn = cr > 0
        elif sgn != (cr > 0):
            return False
    return True


def point_poly_dist(p, poly):
    """Distance from a point to a convex polygon (negative if inside)."""
    d = min(seg_dist(p, a, b) for a, b in zip(poly, poly[1:] + poly[:1]))
    return -d if inside(p, poly) else d


def poly_poly_dist(p1, p2):
    """Min distance between two convex polygons; <= 0 if they overlap."""
    if any(inside(v, p2) for v in p1) or any(inside(v, p1) for v in p2):
        return 0.0 if not all(inside(v, p2) for v in p1) else -1.0
    edges = lambda poly: list(zip(poly, poly[1:] + poly[:1]))  # noqa: E731
    d = min(min(seg_dist(v, a, b) for a, b in edges(p2)) for v in p1)
    return min(d, min(min(seg_dist(v, a, b) for a, b in edges(p1)) for v in p2))


class NavTest:
    def __init__(self, a):
        self.a = a
        self.scenario = rospy.get_param("/pool/scenario")
        self.sc = rospy.get_param("/pool/scenarios/" + self.scenario)
        self.L, self.W = P("pool.length"), P("pool.width")
        self.hl, self.hw = P("vessel.length") / 2, P("vessel.width") / 2
        self.r = math.hypot(self.hl, self.hw)
        self.margin = P("safety.margin")
        self.obstacles = [box_poly(*ob["box"]) for ob in self.sc["obstacles"]]
        self.status, self.trajs, self.planned, self.odom, self.cmds, self.estop = [], [], [], [], [], []
        rospy.Subscriber("/planner/status", String, lambda m: self.status.append((rospy.get_time(), m.data)), queue_size=20)
        rospy.Subscriber("/planner/traj_poly", Polynome, lambda m: self.trajs.append((rospy.get_time(), m)), queue_size=10)
        rospy.Subscriber("/visualizer/mincoPath", Path, self.cb_path, queue_size=10)
        rospy.Subscriber("/pool/robot/odom", Odometry, self.cb_odom, queue_size=200)
        rospy.Subscriber("/pool/cmd_vel", Twist, lambda m: self.cmds.append((rospy.get_time(), m.linear.x, m.angular.z)),
                         queue_size=200)
        rospy.Subscriber("/planner/emergency_stop", Bool, lambda m: self.estop.append(rospy.get_time()), queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)

    def cb_path(self, m):
        self.planned = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def cb_odom(self, m):
        q = m.pose.pose.orientation
        self.odom.append((m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y,
                          euler_from_quaternion([q.x, q.y, q.z, q.w])[2], m.twist.twist.linear.x, m.twist.twist.angular.z))

    # -------------------------------------------------------------- clearance
    def hull_poly(self, x, y, yaw):
        return box_poly(x, y, 2 * self.hl, 2 * self.hw, yaw)

    def hull_clearance(self, x, y, yaw):
        """True hull rectangle: min distance to obstacles and to the pool edge (negative/0 = contact)."""
        hp = self.hull_poly(x, y, yaw)
        walls = min(min(px, self.L - px, py, self.W - py) for px, py in hp)
        obs = min((poly_poly_dist(hp, ob) for ob in self.obstacles), default=float("inf"))
        return min(walls, obs)

    def circle_clearance(self, x, y):
        """Collision circle (radius r): centre distance to obstacles / pool edge minus r."""
        walls = min(x, self.L - x, y, self.W - y)
        obs = min((point_poly_dist((x, y), ob) for ob in self.obstacles), default=float("inf"))
        return min(walls, obs) - self.r

    # -------------------------------------------------------------- run
    def run(self):
        a = self.a
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and (not self.odom or not self.status) and rospy.get_time() - t0 < 30:
            rospy.sleep(0.1)
        rospy.sleep(a.map_settle)                     # laser -> SDF map -> ESDF
        x, y, yaw = self.sc["goal"]
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y = x, y
        q = quaternion_from_euler(0, 0, yaw)
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = q
        n_status0 = len(self.status)
        self.t_goal = rospy.get_time()
        self.goal_pub.publish(g)
        rospy.loginfo("[nav_tests] %s: goal (%.2f, %.2f, %.2f), expect %s", self.scenario, x, y, yaw, self.sc["expect"])
        deadline = self.t_goal + a.timeout
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if any(s in TERMINAL for _, s in self.status[n_status0:]):
                break
            rospy.sleep(0.05)
        rospy.sleep(a.settle)                          # watch the vessel after the outcome
        return self.judge(n_status0)

    def judge(self, n0):
        a, sc = self.a, self.sc
        statuses = [s for _, s in self.status[n0:]]
        odom = [o for o in self.odom if o[0] >= self.t_goal - 0.5]
        rep = {"scenario": self.scenario, "description": sc["description"], "expect": sc["expect"],
               "goal": sc["goal"], "status_sequence": statuses, "collision_radius_m": self.r, "margin_m": self.margin,
               "checks": []}

        def chk(name, ok, detail=""):
            rep["checks"].append({"name": name, "pass": bool(ok), "detail": detail})

        start, end = odom[0], odom[-1]
        moved = math.hypot(end[1] - start[1], end[2] - start[2])
        rep["actual"] = {"start": list(start[1:4]), "end": list(end[1:4]), "displacement_m": moved}
        rep["trajectories_published"] = len(self.trajs)
        if self.trajs:
            m = self.trajs[0][1]
            rep["planned"] = {"pieces": len(m.t_pts), "duration_s": sum(m.t_pts), "arc_length_m": m.tail_p.y,
                              "plan_latency_s": self.trajs[0][0] - self.t_goal}

        if sc["expect"] == "success":
            chk("trajectory published", "TRAJECTORY_PUBLISHED" in statuses, " -> ".join(statuses))
            chk("trajectory completed", "TRAJECTORY_COMPLETE" in statuses)
            pos_err = math.hypot(end[1] - sc["goal"][0], end[2] - sc["goal"][1])
            yaw_err = abs(math.atan2(math.sin(end[3] - sc["goal"][2]), math.cos(end[3] - sc["goal"][2])))
            rep["actual"].update(final_pos_err_m=pos_err, final_yaw_err_rad=yaw_err)
            chk("goal reached (position)", pos_err < a.pos_tol, "%.3f m < %.2f m" % (pos_err, a.pos_tol))
            chk("goal reached (heading)", yaw_err < a.yaw_tol, "%.3f rad < %.2f rad" % (yaw_err, a.yaw_tol))
            hull = min(self.hull_clearance(o[1], o[2], o[3]) for o in odom)
            circ = min(self.circle_clearance(o[1], o[2]) for o in odom)
            rep["actual"].update(min_hull_clearance_m=hull, min_circle_clearance_m=circ)
            chk("actual hull never touches an obstacle or wall", hull > 0.0, "min hull clearance %.3f m" % hull)
            chk("actual collision circle never touches (verification radius r)", circ >= 0.0,
                "min circle clearance %.3f m (margin target %.2f m)" % (circ, self.margin))
            if self.planned:
                pc = min(self.circle_clearance(x, y) for x, y in self.planned)
                rep["planned"]["min_circle_clearance_m"] = pc
                chk("planned path circle clearance >= 0", pc >= 0.0, "%.3f m" % pc)
                errs = [min(math.hypot(o[1] - x, o[2] - y) for x, y in self.planned) for o in odom[::5]]
                rep["actual"]["max_tracking_err_m"] = max(errs)
                chk("tracking error < %.2f m" % a.track_tol, max(errs) < a.track_tol, "%.3f m" % max(errs))
        else:
            fails = [s for s in statuses if s.startswith("FAILED_")]
            chk("failure reported explicitly", bool(fails), " -> ".join(statuses))
            chk("no trajectory published", not self.trajs, "%d published" % len(self.trajs))
            nz = [c for c in self.cmds if c[0] >= self.t_goal and (abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9)]
            chk("no non-zero motion command", not nz, "%d non-zero /pool/cmd_vel" % len(nz))
            chk("vessel stayed stationary", moved < 1e-6, "displacement %.2e m" % moved)
        rep["pass"] = all(c["pass"] for c in rep["checks"])
        self.save(rep, odom)
        return rep

    # -------------------------------------------------------------- output
    def save(self, rep, odom):
        out = self.a.out
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "report.json"), "w") as f:
            json.dump(rep, f, indent=2)
        with open(os.path.join(out, "planned.csv"), "w") as f:
            f.write("x,y\n" + "".join("%.4f,%.4f\n" % p for p in self.planned))
        with open(os.path.join(out, "actual.csv"), "w") as f:
            f.write("t,x,y,yaw,v,omega\n" + "".join("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % o for o in odom))
        with open(os.path.join(out, "cmd.csv"), "w") as f:
            f.write("t,v,omega\n" + "".join("%.3f,%.4f,%.4f\n" % c for c in self.cmds))
        with open(os.path.join(out, "status.csv"), "w") as f:
            f.write("t,status\n" + "".join("%.3f,%s\n" % s for s in self.status))
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib.patches import Polygon as MPoly, Rectangle
        except ImportError:
            return
        fig, ax = plt.subplots(figsize=(13, 7))
        ax.add_patch(Rectangle((0, 0), self.L, self.W, fc=(0.85, 0.92, 1.0), ec="k", lw=2))
        lw, n, outer = P("pool.lanes.lane_width"), int(P("pool.lanes.count")), P("pool.lanes.outer_space")
        for i in range(1, n):
            ax.axhline(outer + i * lw, color="r", ls=":", lw=0.8)
        for ob in self.obstacles:
            ax.add_patch(MPoly(ob, fc=(0.55, 0.15, 0.1), ec="k"))
        if self.planned:
            ax.plot(*zip(*self.planned), "r-", lw=2, label="planned (MINCO)")
        ax.plot([o[1] for o in odom], [o[2] for o in odom], "b--", lw=1.5, label="actual (hull_sim)")
        for o in odom[::100]:
            ax.add_patch(MPoly(self.hull_poly(o[1], o[2], o[3]), fill=False, ec="b", lw=0.6))
        o = odom[-1]
        ax.add_patch(MPoly(self.hull_poly(o[1], o[2], o[3]), fill=False, ec="b", lw=1.5))
        ax.add_patch(plt.Circle((o[1], o[2]), self.r, fill=False, ec="g", ls="--", lw=1, label="collision circle r"))
        gx, gy, gyaw = self.sc["goal"]
        ax.plot(gx, gy, "g*", ms=15, label="goal")
        ax.arrow(gx, gy, 0.8 * math.cos(gyaw), 0.8 * math.sin(gyaw), color="g", width=0.03)
        ax.set_xlim(-0.5, self.L + 0.5)
        ax.set_ylim(-0.5, self.W + 0.5)
        ax.set_aspect("equal")
        ax.set_title("%s (expect %s): %s | %s" % (self.scenario, self.sc["expect"], "PASS" if rep["pass"] else "FAIL",
                                                   " -> ".join(rep["status_sequence"])), fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, self.scenario + ".png"), dpi=90)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/nav_tests")
    ap.add_argument("--timeout", type=float, default=120.0, help="s from goal to terminal status")
    ap.add_argument("--map-settle", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=3.0, help="s to keep watching after the outcome")
    ap.add_argument("--pos-tol", type=float, default=0.3)
    ap.add_argument("--yaw-tol", type=float, default=0.2)
    ap.add_argument("--track-tol", type=float, default=0.3)
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("nav_tests", anonymous=True)
    rep = NavTest(a).run()
    print("\n== %s (expect %s): %s" % (rep["scenario"], rep["expect"], "PASS" if rep["pass"] else "FAIL"))
    print("   status:", " -> ".join(rep["status_sequence"]))
    for c in rep["checks"]:
        print("   [%s] %s %s" % ("PASS" if c["pass"] else "FAIL", c["name"], ("-- " + c["detail"]) if c["detail"] else ""))
    sys.exit(0 if rep["pass"] else 1)


if __name__ == "__main__":
    main()
