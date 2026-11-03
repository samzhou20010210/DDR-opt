#!/usr/bin/env python3
"""Response tests for hull_sim (run against roslaunch pool_rescue pool_hull_sim.launch).

Each test: reset to standby -> publish a timed /pool/cmd_vel schedule at 50 Hz -> record /pool/robot/odom and
/pool/hull_sim/target -> compare measured response with the values predicted from the central config.
Writes <out>/<test>.csv, <out>/<test>.png, <out>/report.json; exit 0 if every check passes.

  rosrun pool_rescue hull_tests.py --out /tmp/hull_tests [--tests straight stop ...]
"""
import argparse
import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

RATE = 50.0          # command publish rate [Hz]
T_TOL = 0.03         # timing tolerance [s]: one 10 ms sim tick + transport + scheduling jitter
REL_TOL = 0.03       # relative tolerance on rates / steady values


def P(path):
    return rospy.get_param("/pool/" + path.replace(".", "/") + "/value")


class Rec:
    def __init__(self):
        self.odom, self.target, self.contact = [], [], []
        rospy.Subscriber("/pool/robot/odom", Odometry, self.cb_odom, queue_size=500, tcp_nodelay=True)
        rospy.Subscriber("/pool/hull_sim/target", TwistStamped, self.cb_tgt, queue_size=500, tcp_nodelay=True)
        rospy.Subscriber("/pool/robot/wall_contact", Bool, self.cb_contact, queue_size=10)

    def cb_odom(self, m):
        q = m.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom.append((m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                          m.twist.twist.linear.x, m.twist.twist.angular.z))

    def cb_tgt(self, m):
        self.target.append((m.header.stamp.to_sec(), m.twist.linear.x, m.twist.angular.z))

    def cb_contact(self, m):
        self.contact.append((rospy.get_time(), m.data))

    def clear(self):
        self.odom, self.target, self.contact = [], [], []


def unwrap(yaws):
    out, off = [yaws[0]], 0.0
    for a, b in zip(yaws[:-1], yaws[1:]):
        d = b - a
        if d > math.pi:
            off -= 2 * math.pi
        elif d < -math.pi:
            off += 2 * math.pi
        out.append(b + off)
    return out


def slope(ts, ys):
    n = len(ts)
    if n < 2:
        return float("nan")
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den > 0 else float("nan")


class Tester:
    def __init__(self, out):
        self.out = out
        self.rec = Rec()
        self.pub = rospy.Publisher("/pool/cmd_vel", Twist, queue_size=10, tcp_nodelay=True)
        rospy.wait_for_service("/hull_sim/reset", 20)
        self.reset_srv = rospy.ServiceProxy("/hull_sim/reset", Empty)
        rospy.wait_for_message("/pool/robot/odom", Odometry, 20)
        mode = rospy.get_param("/pool/mode")
        m = "modes." + mode + "."
        self.cfg = {
            "mode": mode, "v_fwd": P(m + "max_forward_speed"), "v_rev": P(m + "max_reverse_speed"),
            "acc": P(m + "max_acceleration"), "dec": P(m + "max_deceleration"),
            "w_max": P(m + "max_yaw_rate"), "w_acc": P(m + "max_yaw_accel"),
            "latency": P("control.latency"), "timeout": P("control.cmd_timeout"),
            "reverse": bool(P("vessel.reverse_enabled")),
            "L": P("pool.length"), "W": P("pool.width"),
            "hl": P("vessel.length") / 2, "hw": P("vessel.width") / 2,
        }
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.05)

    # ------------------------------------------------------------ running
    def run(self, name, schedule):
        """schedule: list of (duration_s, v, w); v/w None = stop publishing for that segment."""
        self.reset_srv()
        rospy.sleep(0.3)
        self.rec.clear()
        seg_t = []
        r = rospy.Rate(RATE)
        for dur, v, w in schedule:
            t0 = rospy.get_time()
            seg_t.append(t0)
            while not rospy.is_shutdown() and rospy.get_time() - t0 < dur:
                if v is not None:
                    tw = Twist()
                    tw.linear.x, tw.angular.z = v, w
                    self.pub.publish(tw)
                r.sleep()
        rospy.sleep(0.1)
        odom = list(self.rec.odom)
        tgt = list(self.rec.target)
        self.save(name, odom, tgt, seg_t, schedule)
        return odom, tgt, seg_t

    def save(self, name, odom, tgt, seg_t, schedule):
        t0 = seg_t[0]
        with open(os.path.join(self.out, name + ".csv"), "w") as f:
            f.write("t,x,y,yaw,v,omega\n")
            for o in odom:
                f.write("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f\n" % ((o[0] - t0,) + o[1:]))
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        fig, ax = plt.subplots(1, 3, figsize=(15, 4))
        tt = [o[0] - t0 for o in odom]
        # commanded (step) signal
        ct, cv, cw = [], [], []
        for (dur, v, w), ts in zip(schedule, seg_t):
            if v is None:
                continue
            ct += [ts - t0, ts - t0 + dur]
            cv += [v, v]
            cw += [w, w]
        ax[0].plot(ct, cv, "k--", lw=1, label="command")
        ax[0].plot([g[0] - t0 for g in tgt], [g[1] for g in tgt], "c-", lw=1, label="applied target")
        ax[0].plot(tt, [o[4] for o in odom], "r-", label="v (odom)")
        ax[0].set(xlabel="t [s]", ylabel="v [m/s]", title=name + ": speed")
        ax[1].plot(ct, cw, "k--", lw=1, label="command")
        ax[1].plot([g[0] - t0 for g in tgt], [g[2] for g in tgt], "c-", lw=1, label="applied target")
        ax[1].plot(tt, [o[5] for o in odom], "b-", label="omega (odom)")
        ax[1].set(xlabel="t [s]", ylabel="omega [rad/s]", title=name + ": yaw rate")
        ax[2].plot([o[1] for o in odom], [o[2] for o in odom], "g-")
        ax[2].set(xlabel="x [m]", ylabel="y [m]", title=name + ": path")
        ax[2].axis("equal")
        for a in ax[:2]:
            a.grid(True)
            a.legend(fontsize=8)
        ax[2].grid(True)
        fig.tight_layout()
        fig.savefig(os.path.join(self.out, name + ".png"), dpi=90)
        plt.close(fig)

    # ------------------------------------------------------------ helpers
    @staticmethod
    def after(odom, t):
        return [o for o in odom if o[0] >= t]

    @staticmethod
    def first(odom, t, pred):
        for o in odom:
            if o[0] >= t and pred(o):
                return o
        return None

    @staticmethod
    def chk(res, name, measured, expected, tol, unit):
        ok = measured is not None and not math.isnan(measured) and abs(measured - expected) <= tol
        res["checks"].append({"name": name, "measured": measured, "expected": expected, "tol": tol,
                              "unit": unit, "pass": bool(ok)})

    @staticmethod
    def bound(res, name, measured, limit, unit, kind="<="):
        ok = measured is not None and (measured <= limit if kind == "<=" else measured >= limit)
        res["checks"].append({"name": name, "measured": measured, "expected": "%s %s" % (kind, limit),
                              "unit": unit, "pass": bool(ok)})

    def ramp(self, odom, t_from, idx, v0, v1):
        """Slope of odom[idx] over the FIRST contiguous stretch between 10% and 90% of the v0 -> v1 transition
        after t_from (a later return through the same band must not be mixed in)."""
        lo, hi = sorted((v0 + 0.1 * (v1 - v0), v0 + 0.9 * (v1 - v0)))
        pts = []
        for o in self.after(odom, t_from):
            if lo <= o[idx] <= hi:
                pts.append((o[0], o[idx]))
            elif pts:
                break
        return abs(slope([p[0] for p in pts], [p[1] for p in pts])), len(pts)

    # ------------------------------------------------------------ tests
    def t_straight(self):
        c = self.cfg
        cmd = 1.5 * c["v_fwd"]                      # deliberately above the limit -> must clamp
        t_up = c["v_fwd"] / c["acc"]
        odom, _, seg = self.run("straight", [(0.5, 0, 0), (c["latency"] + t_up + 2.0, cmd, 0)])
        res = {"test": "straight", "command": {"v": cmd, "omega": 0}, "checks": []}
        ts = seg[1]
        mv = self.first(odom, ts, lambda o: o[4] > 1e-4)
        self.chk(res, "response latency", mv[0] - ts if mv else None, c["latency"], T_TOL, "s")
        a, n = self.ramp(odom, ts, 4, 0.0, c["v_fwd"])
        self.chk(res, "acceleration", a, c["acc"], REL_TOL * c["acc"], "m/s^2")
        steady = [o[4] for o in odom if o[0] > odom[-1][0] - 1.0]
        self.chk(res, "steady speed (clamped from %.2f)" % cmd, sum(steady) / len(steady), c["v_fwd"], 1e-6, "m/s")
        y0, yaw0 = odom[0][2], odom[0][3]
        self.bound(res, "max lateral deviation", max(abs(o[2] - y0) for o in odom), 1e-6, "m")
        self.bound(res, "max heading drift", max(abs(o[3] - yaw0) for o in odom), 1e-9, "rad")
        res["distance_m"] = odom[-1][1] - odom[0][1]
        return res

    def t_stop(self):
        c = self.cfg
        v = c["v_fwd"]
        odom, _, seg = self.run("stop", [(0.5, 0, 0), (c["latency"] + v / c["acc"] + 1.0, v, 0), (4.0, 0, 0)])
        res = {"test": "stop", "command": {"v": v, "then": 0}, "checks": []}
        ts = seg[2]
        self.chk(res, "speed before stop command", [o for o in odom if o[0] < ts][-1][4], v, 1e-6, "m/s")
        dv = self.first(odom, ts, lambda o: o[4] < v - 1e-4)
        self.chk(res, "response latency", dv[0] - ts if dv else None, c["latency"], T_TOL, "s")
        d, n = self.ramp(odom, ts, 4, v, 0.0)
        self.chk(res, "deceleration", d, c["dec"], REL_TOL * c["dec"], "m/s^2")
        st = self.first(odom, ts, lambda o: o[4] == 0.0)
        self.chk(res, "stopping time (latency + v/dec)", st[0] - ts if st else None,
                 c["latency"] + v / c["dec"], T_TOL, "s")
        at_cmd = [o for o in odom if o[0] < ts][-1]
        dist = math.hypot(odom[-1][1] - at_cmd[1], odom[-1][2] - at_cmd[2])
        exp = v * c["latency"] + v * v / (2 * c["dec"])
        self.chk(res, "stopping distance (v*latency + v^2/2dec)", dist, exp, v * T_TOL + 0.01, "m")
        tail = self.after(odom, st[0]) if st else []
        self.bound(res, "creep after stop", max((abs(o[4]) for o in tail), default=None), 0.0, "m/s")
        return res

    def t_turn(self):
        c = self.cfg
        w = c["w_max"]
        t_up = w / c["w_acc"]
        odom, _, seg = self.run("turn_in_place", [(0.5, 0, 0), (c["latency"] + t_up + 2.0, 0, w), (3.0, 0, 0)])
        res = {"test": "turn_in_place", "command": {"v": 0, "omega": w, "then": 0}, "checks": []}
        ts, te = seg[1], seg[2]
        mw = self.first(odom, ts, lambda o: o[5] > 1e-4)
        self.chk(res, "response latency", mw[0] - ts if mw else None, c["latency"], T_TOL, "s")
        a, _ = self.ramp(odom, ts, 5, 0.0, w)
        self.chk(res, "yaw acceleration", a, c["w_acc"], REL_TOL * c["w_acc"], "rad/s^2")
        steady = [o[5] for o in odom if te - 1.0 < o[0] < te]
        self.chk(res, "steady yaw rate", sum(steady) / len(steady), w, 1e-6, "rad/s")
        d, _ = self.ramp(odom, te, 5, w, 0.0)
        self.chk(res, "yaw deceleration", d, c["w_acc"], REL_TOL * c["w_acc"], "rad/s^2")
        x0, y0 = odom[0][1], odom[0][2]
        self.bound(res, "position drift (turning in place)", max(math.hypot(o[1] - x0, o[2] - y0) for o in odom),
                   1e-6, "m")
        yaws = unwrap([o[3] for o in odom])
        res["yaw_change_rad"] = yaws[-1] - yaws[0]
        self.bound(res, "turns counter-clockwise for omega > 0", res["yaw_change_rad"], 0.0, "rad", ">=")
        return res

    def t_arc(self):
        c = self.cfg
        v, w = 0.5 * c["v_fwd"], 0.5 * c["w_max"]
        odom, _, seg = self.run("arc_turn", [(0.5, 0, 0), (c["latency"] + 4.0, v, w)])
        res = {"test": "arc_turn", "command": {"v": v, "omega": w}, "checks": []}
        t_ss = seg[1] + c["latency"] + max(v / c["acc"], w / c["w_acc"]) + 0.2
        ss = self.after(odom, t_ss)
        dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(ss[:-1], ss[1:]))
        dyaw = unwrap([o[3] for o in ss])
        radius = dist / (dyaw[-1] - dyaw[0])
        self.chk(res, "steady turn radius (v/omega)", radius, v / w, 0.01 * v / w, "m")
        return res

    def t_reverse(self, name="reverse"):
        c = self.cfg
        v_cmd = -c["v_rev"] if c["v_rev"] > 0 else -0.3
        fwd = c["v_fwd"] / c["acc"] + 1.5
        sched = [(0.5, 0, 0), (fwd, c["v_fwd"], 0), (c["latency"] + c["v_fwd"] / c["dec"] + 0.5, 0, 0),
                 (c["latency"] + abs(v_cmd) / c["acc"] + 1.5, v_cmd, 0), (2.0, 0, 0)]
        odom, _, seg = self.run(name, sched)
        res = {"test": name, "reverse_enabled": c["reverse"], "command": {"v": v_cmd}, "checks": []}
        tr, te = seg[3], seg[4]
        rev = [o for o in odom if tr <= o[0] < te]
        vmin = min(o[4] for o in rev)
        if c["reverse"]:
            self.chk(res, "reverse speed reached", vmin, -c["v_rev"], 1e-6, "m/s")
            mv = self.first(odom, tr, lambda o: o[4] < -1e-4)
            self.chk(res, "response latency", mv[0] - tr if mv else None, c["latency"], T_TOL, "s")
            a, _ = self.ramp(odom, tr, 4, 0.0, -c["v_rev"])
            self.chk(res, "reverse acceleration", a, c["acc"], REL_TOL * c["acc"], "m/s^2")
            x_at = [o for o in odom if o[0] < tr][-1][1]
            self.bound(res, "moved backwards (x decreased)", odom[-1][1] - x_at, -0.1, "m")
        else:
            self.chk(res, "no backward speed (reverse disabled)", vmin, 0.0, 1e-9, "m/s")
            x_at = [o for o in odom if o[0] < tr][-1][1]
            self.chk(res, "no backward motion", odom[-1][1] - x_at, 0.0, 1e-9, "m")
        return res

    def t_wall(self):
        c = self.cfg
        # from standby (facing +x near y=0 wall) turn right at full rate -> the hull must hit the y=0 wall
        sched = [(0.5, 0, 0), (6.0, 0.5 * c["v_fwd"], -c["w_max"])]
        if c["reverse"]:
            sched.append((2.0, -c["v_rev"], 0.0))        # recovery: back away from the wall
        odom, _, seg = self.run("wall_contact", sched)
        res = {"test": "wall_contact", "command": {"v": 0.5 * c["v_fwd"], "omega": -c["w_max"],
                                                   "then_v": -c["v_rev"] if c["reverse"] else None}, "checks": []}
        hl, hw = c["hl"], c["hw"]

        def margin(o):
            cs, sn = math.cos(o[3]), math.sin(o[3])
            d = 1e9
            for bx, by in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
                x, y = o[1] + cs * bx - sn * by, o[2] + sn * bx + cs * by
                d = min(d, x, c["L"] - x, y, c["W"] - y)
            return d
        res["min_hull_wall_margin_m"] = min(margin(o) for o in odom)
        self.bound(res, "hull never outside the pool", res["min_hull_wall_margin_m"], 0.0, "m", ">=")
        hits = [t for t, cv in self.rec.contact if cv]
        self.bound(res, "wall_contact reported", 1.0 if hits else 0.0, 1.0, "-", ">=")
        if hits:
            t_hit = hits[0]
            t_end = seg[2] + c["latency"] if c["reverse"] else odom[-1][0] + 1
            held = [o for o in odom if t_hit + 0.02 <= o[0] < t_end]
            self.bound(res, "no motion while the same command pushes into the wall",
                       max((abs(o[4]) + abs(o[5]) for o in held), default=None), 0.0, "m/s + rad/s")
            res["contact_time_s"] = t_hit - seg[1]
        if c["reverse"]:
            x_at, y_at = [(o[1], o[2]) for o in odom if o[0] < seg[2]][-1]
            self.bound(res, "recovers by reversing away", math.hypot(odom[-1][1] - x_at, odom[-1][2] - y_at),
                       0.05, "m", ">=")
            self.bound(res, "contact cleared after recovery", 0.0 if self.rec.contact[-1][1] else 1.0, 1.0, "-", ">=")
        return res

    def t_timeout(self):
        c = self.cfg
        v = c["v_fwd"]
        odom, _, seg = self.run("cmd_timeout", [(0.5, 0, 0), (c["latency"] + v / c["acc"] + 0.5, v, 0),
                                                (c["timeout"] + c["latency"] + v / c["dec"] + 1.0, None, None)])
        res = {"test": "cmd_timeout", "checks": []}
        t_last = seg[2]                                  # publishing stops here
        dv = self.first(odom, t_last, lambda o: o[4] < v - 1e-4)
        self.chk(res, "decel starts after latency + timeout", dv[0] - t_last if dv else None,
                 c["latency"] + c["timeout"], T_TOL + 1.0 / RATE, "s")
        self.chk(res, "stopped", odom[-1][4], 0.0, 1e-9, "m/s")
        return res


TESTS = {"straight": "t_straight", "stop": "t_stop", "turn": "t_turn", "arc": "t_arc",
         "reverse": "t_reverse", "wall": "t_wall", "timeout": "t_timeout"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/hull_tests")
    ap.add_argument("--tests", nargs="+", default=list(TESTS), choices=list(TESTS))
    a = ap.parse_args(rospy.myargv()[1:])
    os.makedirs(a.out, exist_ok=True)
    rospy.init_node("hull_tests", anonymous=True)
    t = Tester(a.out)
    results = []
    for name in a.tests:
        res = getattr(t, TESTS[name])()
        res["pass"] = all(ch["pass"] for ch in res["checks"])
        results.append(res)
        print("\n== %s: %s" % (res["test"], "PASS" if res["pass"] else "FAIL"))
        for ch in res["checks"]:
            m = ch["measured"]
            print("  [%s] %-45s measured %-12s expected %s%s %s" % (
                "PASS" if ch["pass"] else "FAIL", ch["name"],
                ("%.4f" % m) if isinstance(m, float) else str(m),
                ch["expected"] if isinstance(ch["expected"], str) else "%.4f" % ch["expected"],
                (" +/- %.4f" % ch["tol"]) if "tol" in ch else "", ch["unit"]))
    report = {"config": t.cfg, "results": results, "pass": all(r["pass"] for r in results)}
    with open(os.path.join(a.out, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print("\nRESULT:", "PASS" if report["pass"] else "FAIL", "| artifacts:", a.out)
    sys.exit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
