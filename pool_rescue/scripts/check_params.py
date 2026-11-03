#!/usr/bin/env python3
"""Validate the pool rescue configuration.

Offline (default): central file schema + sanity, generated files up to date.
Live (--live, needs a running pool_sim.launch): every legacy node really received the
generated values (incl. mpc_controller's hard-coded /mpc/ namespace), /pool params loaded,
each TF frame has exactly one publisher, interface topics have the agreed types/publishers.

  python3 check_params.py                       # offline
  rosrun pool_rescue check_params.py --live --mode approach
Exit code 0 = all checks passed.
"""
import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_ddr_params as gen  # noqa: E402

UNITS = {"m", "m/s", "m/s^2", "rad", "rad/s", "rad/s^2", "s", "Hz", "-"}
SOURCES = ("stock:", "standard:", "assumption")

# Every key mpc_controller reads (mpc.cpp constructor, absolute /mpc/ namespace; default -1 if missing).
MPC_KEYS = ["du_threshold", "dt", "max_iter", "predict_steps", "max_omega", "max_domega", "max_vel",
            "min_vel", "max_acc", "delay_num", "cmd_timer_rate", "max_mpc_time", "if_mpc",
            "matrix_q", "matrix_r", "matrix_rd"]

# Per launch stack: legacy nodes whose generated params must be applied, agreed interfaces
# (topic -> (type, single publisher)), TF owners (child frame -> single publisher), nodes that must NOT run.
STACKS = {
    "ddr": {   # pool_sim.launch: legacy DDR-opt simulator + EKF + ddr_adapter
        "nodes": ["global_planning", "simulator", "mpc", "global_map_node", "laser_simulator", "icrekf", "ddr_adapter"],
        "interfaces": {
            "/pool/robot/odom": ("nav_msgs/Odometry", "/ddr_adapter"),
            "/pool/robot/odom_truth": ("nav_msgs/Odometry", "/ddr_adapter"),
            "/pool/cmd_vel": ("geometry_msgs/Twist", "/ddr_adapter"),
            "/planner/traj_poly": ("carstatemsgs/Polynome", "/global_planning"),
            "/planner/status": ("std_msgs/String", "/global_planning"),
        },
        "tf_owners": {"base_link": "/ddr_adapter", "laser": "/laser_simulator"},
        "absent": [],
    },
    "nav": {   # pool_nav.launch: DDR-opt planner + MPC driving hull_sim via nav_bridge
        "nodes": ["global_planning", "mpc", "global_map_node", "laser_simulator"],
        "interfaces": {
            "/pool/robot/odom": ("nav_msgs/Odometry", "/hull_sim"),
            "/pool/cmd_vel": ("geometry_msgs/Twist", "/nav_bridge"),
            "/pool/internal/sensor_pose": ("carstatemsgs/CarState", "/nav_bridge"),
            "/planner/traj_poly": ("carstatemsgs/Polynome", "/global_planning"),
            "/planner/status": ("std_msgs/String", "/global_planning"),
        },
        "tf_owners": {"base_link": "/hull_sim", "laser": "/laser_simulator"},
        "absent": ["/simulator", "/icrekf", "/ddr_adapter"],
    },
}


class Report:
    def __init__(self):
        self.rows = []

    def check(self, ok, name, detail=""):
        self.rows.append((bool(ok), name, detail))
        return ok

    def done(self):
        for ok, name, detail in self.rows:
            print("[{}] {}{}".format("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
        n_fail = sum(1 for r in self.rows if not r[0])
        print("RESULT: {} ({} checks, {} failed)".format("PASS" if n_fail == 0 else "FAIL", len(self.rows), n_fail))
        return n_fail == 0


def leaves(node, path=""):
    if isinstance(node, dict) and "value" in node:
        yield path, node
    elif isinstance(node, dict):
        for k, child in node.items():
            yield from leaves(child, path + "." + k if path else k)
    else:
        yield path, None


def offline(rep):
    cfg = gen.load_central()
    v = lambda p: gen.v(cfg, p)  # noqa: E731

    # 1. schema: every leaf annotated
    n = 0
    for path, leaf in leaves(cfg):
        n += 1
        if leaf is None:
            rep.check(False, "annotated: " + path, "not a {value, unit, source} leaf")
            continue
        unit, src = leaf.get("unit"), str(leaf.get("source", ""))
        if unit not in UNITS or not src.startswith(SOURCES):
            rep.check(False, "annotated: " + path, "unit={!r} source={!r}".format(unit, src))
    rep.check(not any(not r[0] for r in rep.rows), "all {} central parameters have value/unit/source".format(n))

    # 2. sanity
    for mode in gen.MODES:
        for k in ("max_forward_speed", "max_reverse_speed", "max_acceleration", "max_deceleration",
                  "max_yaw_rate", "max_yaw_accel", "max_centripetal_acc"):
            rep.check(v("modes.%s.%s" % (mode, k)) > 0, "positive: modes.%s.%s" % (mode, k))
    for k in ("max_forward_speed", "max_reverse_speed", "max_acceleration", "max_deceleration",
              "max_yaw_rate", "max_yaw_accel", "max_centripetal_acc"):
        rep.check(v("modes.rescue." + k) <= v("modes.approach." + k), "rescue <= approach: " + k)

    L, W = v("pool.length"), v("pool.width")
    hl, hw = v("vessel.length") / 2, v("vessel.width") / 2
    reach = math.hypot(hl, hw)
    sx, sy = v("standby.x"), v("standby.y")
    margin = min(sx, sy, L - sx, W - sy) - reach - v("pool.wall_thickness_in_map") / 2
    rep.check(margin >= v("safety.margin"), "standby pose: hull clear of walls by the safety margin",
              "worst-case hull-wall clearance {:.2f} m >= {:.2f} m".format(margin, v("safety.margin")))
    clr = gen.clearances(cfg)
    centre = min(sx, sy, L - sx, W - sy) - v("pool.wall_thickness_in_map") / 2
    rep.check(centre >= clr["planning"], "standby centre plannable (>= r + m from walls in the map)",
              "centre clearance {:.3f} m >= r + m = {:.3f} m (r = {:.3f}, m = {:.2f})".format(
                  centre, clr["planning"], clr["collision_radius"], clr["margin"]))
    lanes = v("pool.lanes.count") * v("pool.lanes.lane_width") + 2 * v("pool.lanes.outer_space")
    rep.check(abs(lanes - W) < 1e-9, "lanes fill the pool width", "{} x {} + 2 x {} = {} m (pool {} m)".format(
        v("pool.lanes.count"), v("pool.lanes.lane_width"), v("pool.lanes.outer_space"), lanes, W))
    for name, sc in gen.load_scenarios().items():
        g = sc.get("goal")
        if g:
            rep.check(0 < g[0] < L and 0 < g[1] < W, "scenario %s: goal inside pool" % name, str(g))
    ez = [v("extraction_zone." + k) for k in ("x", "y")]
    rep.check(0 < ez[0] < L and 0 < ez[1] < W, "extraction zone centre inside pool", str(ez))

    lat, dt = v("control.latency"), v("control.mpc_dt")
    steps = int(round(lat / dt))
    rep.check(abs(steps * dt - lat) <= dt / 2 + 1e-9, "latency representable in MPC steps",
              "delay_num={} x dt={} = {:.3f} s".format(steps, dt, steps * dt))

    # 3. generated files current
    stale = [os.path.relpath(p_, gen.PKG) for p_ in gen.stale_files(cfg)]
    rep.check(not stale, "generated files up to date", ", ".join(stale) or "")
    return cfg


def same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


def live(rep, mode, wait, stack="ddr", scenario=None):
    import rosgraph
    import rospy
    from tf2_msgs.msg import TFMessage

    rospy.init_node("param_check", anonymous=True, disable_signals=True)
    master = rosgraph.Master("/param_check")

    # wait until the adapter is publishing TF (EKF up -> whole chain up)
    tf_pubs = {}

    def tf_cb(msg):
        caller = msg._connection_header.get("callerid", "?")
        for t in msg.transforms:
            tf_pubs.setdefault(t.child_frame_id, {}).setdefault(t.header.frame_id, set()).add(caller)

    rospy.Subscriber("/tf", TFMessage, tf_cb, queue_size=100)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and "base_link" not in tf_pubs and rospy.get_time() - t0 < wait:
        rospy.sleep(0.2)
    rospy.sleep(3.0)  # collect TF from every publisher for a while

    # 1. every legacy node received the generated values
    state = master.getSystemState()  # (publishers, subscribers, services)
    nodes = {n for section in state for _, ns in section for n in ns}
    spec = STACKS[stack]
    for name in spec["nodes"]:
        node = "/" + name
        want = yaml.safe_load(open(os.path.join(gen.OUT_DIR, mode, name + ".yaml")))
        if name == "global_map_node" and scenario:
            want.update(yaml.safe_load(open(os.path.join(gen.OUT_DIR, "scenarios", scenario + ".yaml"))))
        rep.check(node in nodes, "node running: " + node)
        bad = []
        for k, val in want.items():
            key = node + "/" + k
            if not rospy.has_param(key):
                bad.append(k + " missing")
            elif not same(rospy.get_param(key), val):
                bad.append("{}={!r} (want {!r})".format(k, rospy.get_param(key), val))
        rep.check(not bad, "params applied: {} ({} keys)".format(node, len(want)), "; ".join(bad))

    for node in spec["absent"]:
        rep.check(node not in nodes, "node NOT running (disabled in this stack): " + node)

    # 2. mpc_controller's hard-coded /mpc namespace
    missing = [k for k in MPC_KEYS if not rospy.has_param("/mpc/" + k)]
    rep.check(not missing, "/mpc/* has every key mpc_controller reads (else it silently uses -1)",
              ", ".join(missing))

    # 3. central params under /pool
    rep.check(rospy.get_param("/pool/mode", None) == mode, "/pool/mode == " + mode)
    cfg = gen.load_central()
    n_bad = sum(1 for p, leaf in leaves(cfg)
                if leaf and not same(rospy.get_param("/pool/" + p.replace(".", "/") + "/value", None), leaf["value"]))
    rep.check(n_bad == 0, "/pool/<group>/<name>/value loaded for all central params", "%d mismatched" % n_bad)

    # 4. TF: one publisher per frame
    for child, parents in sorted(tf_pubs.items()):
        callers = set().union(*parents.values())
        rep.check(len(parents) == 1 and len(callers) == 1, "TF single publisher: {} -> {}".format(
            "/".join(parents), child), "publishers: " + ", ".join(sorted(callers)))
    for child, owner in spec["tf_owners"].items():
        got = set().union(*tf_pubs.get(child, {}).values()) if child in tf_pubs else set()
        parent = list(tf_pubs.get(child, {"-": None}))[0]
        rep.check(got == {owner} and parent == "world", "TF world -> {} owned by {}".format(child, owner),
                  "parent={} publishers={}".format(parent, sorted(got)))

    # 5. interface topics: type and single publisher
    types = dict(master.getTopicTypes())
    pubs = {t: set(ns) for t, ns in master.getSystemState()[0]}
    for topic, (typ, owner) in spec["interfaces"].items():
        rep.check(types.get(topic) == typ and pubs.get(topic, set()) == {owner},
                  "interface {} : {}".format(topic, typ),
                  "type={} publishers={}".format(types.get(topic), sorted(pubs.get(topic, []))))
    try:
        from pool_rescue.msg import Person, PersonArray, RescueTask, ExtractionZone  # noqa: F401
        rep.check(True, "pool_rescue messages importable (Person, PersonArray, RescueTask, ExtractionZone)")
    except ImportError as e:
        rep.check(False, "pool_rescue messages importable", str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--mode", default="approach", choices=gen.MODES)
    ap.add_argument("--stack", default="ddr", choices=sorted(STACKS), help="live: which launch stack is running")
    ap.add_argument("--scenario", default=None, help="live, nav stack: scenario whose map_box must be loaded")
    ap.add_argument("--wait", type=float, default=30.0, help="live: max seconds to wait for the stack")
    try:
        import rospy
        argv = rospy.myargv()[1:]
    except ImportError:
        argv = sys.argv[1:]
    a = ap.parse_args(argv)
    rep = Report()
    offline(rep)
    if a.live:
        live(rep, a.mode, a.wait, a.stack, a.scenario)
    sys.exit(0 if rep.done() else 1)


if __name__ == "__main__":
    main()
