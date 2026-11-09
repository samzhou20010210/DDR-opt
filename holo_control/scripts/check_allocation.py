#!/usr/bin/env python3
"""Offline check of the ROV V4 thruster geometry used by robot:=rov_holo (numpy only, no ROS needed).

1. Exactness of the planner's allocation-reachability constraint.
   The planner requires the planar wrench tau = (X, Y, N) to satisfy 12 linear inequalities n_k . tau <= h_k. This
   script compares that test with an exact per-thruster feasibility test (does a thrust vector f with
   f_min <= f_i <= f_max and B_h f = tau exist?) on random wrenches. B_h is 3 x 4 with rank 3, so its solutions are
   f = f_p + t n0, and the test reduces to intersecting 4 intervals in t.
2. What the reserve means per thruster. The facet offsets are shrunk by `reserve` x the facet half-range. That is
   the zonotope scaled by (1 - reserve) about its centre, which equals every horizontal thruster limited to
   centre +- (1 - reserve) x half-range.
3. Whether the vertical thrusters can always hold Z = K (roll) = M (pitch) = 0. The horizontal thrusters sit at
   z = 0.045 m, so their thrust creates roll and pitch moments; the 4 vertical thrusters must cancel them. That
   requirement is linear in the horizontal thrusts, and the set the verticals can produce is convex, so checking
   the 16 corners of the horizontal thrust box covers every possible horizontal command.

  python3 check_allocation.py [--thrusters rov_thrusters.yaml] [--holo robot_rov_holo.yaml] [--samples N]
"""
import argparse
import itertools
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "..", "plan_manager", "config")


def interval(fp, n0, lo, hi):
    """t-interval for which lo <= fp + t n0 <= hi holds componentwise (empty if t_lo > t_hi)."""
    t_lo, t_hi = -np.inf, np.inf
    for a, b in zip(fp, n0):
        if abs(b) < 1e-12:
            if a < lo - 1e-9 or a > hi + 1e-9:
                return 1.0, -1.0
            continue
        t1, t2 = (lo - a) / b, (hi - a) / b
        t_lo, t_hi = max(t_lo, min(t1, t2)), min(t_hi, max(t1, t2))
    return t_lo, t_hi


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--thrusters", default=os.path.join(CFG, "rov_thrusters.yaml"))
    p.add_argument("--holo", default=os.path.join(CFG, "robot_rov_holo.yaml"))
    p.add_argument("--samples", type=int, default=200000)
    a = p.parse_args()
    th = yaml.safe_load(open(a.thrusters))["thrusters"]
    reserve = yaml.safe_load(open(a.holo))["holo"]["alloc"]["reserve"]
    fmin, fmax = th["f_min"], th["f_max"]
    r = np.array([th["x"], th["y"], th["z"]]).T
    d = np.array([th["axis_x"], th["axis_y"], th["axis_z"]]).T
    B = np.vstack([d.T, np.cross(r, d).T])                         # 6 x 8: X Y Z K M N
    hor = [i for i in range(len(r)) if abs(d[i, 2]) < 0.5]
    ver = [i for i in range(len(r)) if abs(d[i, 2]) >= 0.5]
    print("B (rows X Y Z K M N, columns in CSV order), rank %d:" % np.linalg.matrix_rank(B))
    print(np.array2string(B, precision=4, suppress_small=True))

    # ---- 1. facets (same construction as HoloPlanner::buildAllocationFacets) vs exact test
    Bh = B[[0, 1, 5]][:, hor]
    c_f, h_f = 0.5 * (fmax + fmin), 0.5 * (fmax - fmin)
    centre = Bh.sum(axis=1) * c_f
    gens = [Bh[:, i] * h_f for i in range(len(hor))]

    def facets(res):
        out = []
        for i, j in itertools.combinations(range(len(gens)), 2):
            n = np.cross(gens[i], gens[j])
            if np.linalg.norm(n) < 1e-9:
                continue
            n /= np.linalg.norm(n)
            rr = sum(abs(n @ g) for g in gens)
            for s in (-1, 1):
                out.append((s * n / rr, (s * n @ centre + (1 - res) * rr) / rr))
        return out

    n0 = np.linalg.svd(Bh)[2][-1]
    pinv = np.linalg.pinv(Bh)

    def exact(tau, lo, hi):
        t_lo, t_hi = interval(pinv @ tau, n0, lo, hi)
        return t_lo <= t_hi + 1e-12

    rng = np.random.default_rng(0)
    ext = np.abs(Bh).sum(axis=1) * max(abs(fmin), abs(fmax)) * 1.1
    for res in (0.0, reserve):
        F = facets(res)
        lo, hi = c_f - (1 - res) * h_f, c_f + (1 - res) * h_f
        mism, inside, worst = 0, 0, 0.0
        for _ in range(a.samples):
            tau = rng.uniform(-ext, ext)
            v = max(n @ tau - h for n, h in F)
            e = exact(tau, lo, hi)
            inside += e
            if (v <= 0) != e:
                mism += 1
                worst = max(worst, abs(v))
        print("\n[1] reserve %.2f: %d facets; per-thruster range [%.1f, %.1f] N; %d random wrenches, %d inside; "
              "facet test disagrees with the exact LP-equivalent test on %d (max |margin| at a disagreement %.1e)"
              % (res, len(F), lo, hi, a.samples, inside, mism, worst))
        sup = [max(s * (Bh[k] @ np.array(f)) for f in itertools.product((lo, hi), repeat=len(hor)) for s in (1,))
               for k in range(3)]
        inf = [min(Bh[k] @ np.array(f) for f in itertools.product((lo, hi), repeat=len(hor))) for k in range(3)]
        print("    reachable alone: X [%.1f, %.1f] N, Y [%.1f, %.1f] N, N [%.1f, %.1f] N m"
              % (inf[0], sup[0], inf[1], sup[1], inf[2], sup[2]))

    # ---- 3. vertical thrusters cancelling Z, K, M from every corner of the horizontal box
    Bv = B[[2, 3, 4]][:, ver]
    Bhz = B[[2, 3, 4]][:, hor]
    nv = np.linalg.svd(Bv)[2][-1]
    pv = np.linalg.pinv(Bv)
    worst_need, worst_corner, feasible = 0.0, None, True
    for corner in itertools.product((fmin, fmax), repeat=len(hor)):
        rhs = -Bhz @ np.array(corner)
        fp = pv @ rhs
        t_lo, t_hi = interval(fp, nv, fmin, fmax)
        if t_lo > t_hi:
            feasible = False
        ts = np.linspace(max(t_lo, -1e3), min(t_hi, 1e3), 2001) if t_lo <= t_hi else np.linspace(-100, 100, 20001)
        need = min(np.abs(fp + t * nv).max() for t in ts)
        if need > worst_need:
            worst_need, worst_corner = need, (corner, rhs)
    print("\n[3] vertical thrusters (Z, K, M) = 0 against the roll/pitch moments of the horizontal thrusters:")
    print("    worst horizontal corner %s -> K %.2f N m, M %.2f N m to cancel; smallest max |vertical thrust| that "
          "does it: %.2f N (limits [%.1f, %.1f]) -> %s"
          % (worst_corner[0], -worst_corner[1][1], -worst_corner[1][2], worst_need, fmin, fmax,
             "always feasible" if feasible else "NOT always feasible"))


if __name__ == "__main__":
    main()
