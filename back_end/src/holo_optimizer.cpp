#include "back_end/holo_optimizer.h"

#include <algorithm>
#include <cmath>

// ------------------------------------------------------------------------------------------------ setup

HoloPlanner::HoloPlanner(ros::NodeHandle &nh, std::shared_ptr<SDFmap> map) : nh_(nh), map_(map)
{
    // Limits (PLACEHOLDERS, see robot_rov_holo.yaml)
    param("holo/max_vx", lim_.max_vx, 1.0);
    param("holo/min_vx", lim_.min_vx, -0.3);
    param("holo/max_vy", lim_.max_vy, 0.5);
    param("holo/max_omega", lim_.max_omega, 0.8);
    param("holo/max_ax", lim_.max_ax, 0.4);
    param("holo/max_ay", lim_.max_ay, 0.3);
    param("holo/max_domega", lim_.max_domega, 1.0);

    param("holo/weights/time", wt_.time, 50.0);
    param("holo/weights/vel", wt_.vel, 300.0);
    param("holo/weights/omega", wt_.omega, 300.0);
    param("holo/weights/acc", wt_.acc, 300.0);
    param("holo/weights/domega", wt_.domega, 300.0);
    param("holo/weights/collision", wt_.collision, 500000.0);
    param("holo/weights/alloc", wt_.alloc, 300.0);
    param("holo/weights/mean_time", wt_.mean_time, 300.0);
    wt_backup_ = wt_;

    std::vector<double> ew;
    param("holo/energy_weights", ew, std::vector<double>{1.0, 1.0, 0.3});
    energy_w_ = Eigen::Vector3d(ew.at(0), ew.at(1), ew.at(2));

    // Shared with MSPlanner (same yaml keys): sampling, clearance, time balance, L-BFGS
    param("sparseResolution", K_, 8);
    param("smoothingFactor", smooth_eps_, 0.01);
    param("mean_time_lowBound", mean_time_low_, 0.5);
    param("mean_time_uppBound", mean_time_upp_, 2.0);
    param("safeDis", safe_dis_cfg_, 0.3);
    safe_dis_ = safe_dis_cfg_;
    param("finalMinSafeDis", final_min_safe_dis_, 0.1);
    param("finalSafeDisCheckNum", final_check_num_, 16);
    param("safeReplanMaxTime", safe_replan_max_, 3);
    param("trajPredictResolution", predict_res_, 0.01);
    param("mintrajNum", mintraj_num_, 3);
    param("holo/piece_length", piece_len_, 0.6);
    param("holo/limit_tolerance", limit_tol_, 0.0);
    param("holo/limit_margin", limit_margin_, 0.05);
    param("holo/max_attempts", max_attempts_, 6);
    param("holo/gradient_check", gradient_check_, false);

    int mem = 256, maxit = 8000, past = 3;
    double delta = 5e-3;
    param("lbfgs_params/mem_size", mem, mem);
    param("lbfgs_params/past", past, past);
    param("lbfgs_params/delta", delta, delta);
    param("lbfgs_params/max_iterations", maxit, maxit);
    lbfgs_params_.mem_size = mem;
    lbfgs_params_.past = past;
    lbfgs_params_.delta = delta;
    lbfgs_params_.max_iterations = maxit;
    lbfgs_params_.g_epsilon = 0.0;
    lbfgs_params_.min_step = 1e-32;

    std::vector<double> cp;
    param("checkpoint", cp, std::vector<double>{0.0, 0.0});
    for (size_t i = 0; i + 1 < cp.size(); i += 2) check_points_.emplace_back(cp[i], cp[i + 1]);

    // Placeholder dynamics for the allocation-reachability constraint
    param("holo/alloc/enable", alloc_enable_, true);
    param("holo/alloc/mass", mass_, 60.0);
    param("holo/alloc/inertia_z", inertia_z_, 6.0);
    std::vector<double> dr;
    param("holo/alloc/linear_drag", dr, std::vector<double>{60.0, 90.0, 15.0});
    drag_ = Eigen::Vector3d(dr.at(0), dr.at(1), dr.at(2));
    buildAllocationFacets();

    ROS_WARN("[holo] planar holonomic planner: limits vx [%.2f, %.2f] |vy| %.2f |w| %.2f |ax| %.2f |ay| %.2f |dw| %.2f "
             "(PLACEHOLDERS), allocation constraint %s (placeholder M/D/thrust), %zu facets",
             lim_.min_vx, lim_.max_vx, lim_.max_vy, lim_.max_omega, lim_.max_ax, lim_.max_ay, lim_.max_domega,
             alloc_enable_ ? "ON" : "OFF", facet_n_.size());
}

// Planar wrench tau = (X, Y, N) reachable by the horizontal thrusters: tau = B f, f_min <= f <= f_max.
// This set is a zonotope; its facets come from pairs of generators (n = g_i x g_j). Shrunk by alloc_reserve
// around its centre so the controller keeps a margin for disturbance rejection.
void HoloPlanner::buildAllocationFacets()
{
    facet_n_.clear();
    facet_h_.clear();
    std::vector<double> tx, ty, ax, ay, az;
    double fmax, fmin, reserve;
    param("thrusters/x", tx, std::vector<double>());
    param("thrusters/y", ty, std::vector<double>());
    param("thrusters/axis_x", ax, std::vector<double>());
    param("thrusters/axis_y", ay, std::vector<double>());
    param("thrusters/axis_z", az, std::vector<double>());
    param("thrusters/f_max", fmax, 40.0);
    param("thrusters/f_min", fmin, -30.0);
    param("holo/alloc/reserve", reserve, 0.2);
    if (tx.empty() || tx.size() != ty.size() || tx.size() != ax.size() || tx.size() != ay.size() || tx.size() != az.size())
    {
        ROS_ERROR("[holo] thruster layout missing or inconsistent: allocation constraint disabled");
        alloc_enable_ = false;
        return;
    }
    std::vector<Eigen::Vector3d> gens;
    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < tx.size(); k++)
    {
        if (std::fabs(az[k]) > 0.5) continue;  // vertical thruster: no planar contribution
        const Eigen::Vector3d b(ax[k], ay[k], tx[k] * ay[k] - ty[k] * ax[k]);  // (X, Y, N) per newton of thrust
        centre += b * 0.5 * (fmax + fmin);
        gens.push_back(b * 0.5 * (fmax - fmin));
    }
    for (size_t i = 0; i < gens.size(); i++)
        for (size_t j = i + 1; j < gens.size(); j++)
        {
            Eigen::Vector3d n = gens[i].cross(gens[j]);
            if (n.norm() < 1e-9) continue;
            n.normalize();
            double r = 0.0;
            for (const auto &g : gens) r += std::fabs(n.dot(g));
            // normalised by the facet's half-range r, so the violation is a dimensionless fraction of the thrust
            // range (the smoothed-L1 corner, smoothingFactor, then means the same as for the other terms)
            for (int sgn = -1; sgn <= 1; sgn += 2)
            {
                facet_n_.push_back(sgn * n / r);
                facet_h_.push_back((sgn * n.dot(centre) + (1.0 - reserve) * r) / r);
            }
        }
    if (facet_n_.empty()) alloc_enable_ = false;
}

// ------------------------------------------------------------------------------------------------ main entry

// Holonomic initial guess. Only the JPS path geometry is used: the front end's (yaw, s, t) samples follow the
// differential-drive pattern (rotate in place, then drive) with matching timing, which made every replan start slowly.
// Here the path is resampled uniformly by arc length, the heading follows the path tangent (surge is the faster axis;
// the optimizer may still sway / crab), unwrapped from the start yaw, and the timing comes from the surge limits.
bool HoloPlanner::initialGuess(const FlatTrajData &flat, const PVA &head)
{
    std::vector<Eigen::Vector2d> poly{head.col(0).head<2>()};
    for (const auto &q : flat.UnOccupied_positions)
        if ((q.head<2>() - poly.back()).norm() > 0.05) poly.push_back(q.head<2>());
    const Eigen::Vector2d goal = flat.final_state_XYTheta.head<2>();
    if ((goal - poly.back()).norm() > 0.05 || poly.size() == 1) poly.push_back(goal);
    else poly.back() = goal;
    std::vector<double> cum{0.0};
    for (size_t k = 1; k < poly.size(); k++) cum.push_back(cum.back() + (poly[k] - poly[k - 1]).norm());
    const double L = cum.back();

    N_ = std::max(mintraj_num_, int(std::ceil(L / piece_len_)));
    inner_.resize(3, N_ - 1);
    double yaw_prev = head(2, 0);
    size_t seg = 1;
    for (int i = 1; i < N_; i++)
    {
        const double sl = L * i / N_;
        while (seg + 1 < poly.size() && cum[seg] < sl) seg++;
        const double f = (cum[seg] - cum[seg - 1]) > 1e-9 ? (sl - cum[seg - 1]) / (cum[seg] - cum[seg - 1]) : 0.0;
        const Eigen::Vector2d pt = poly[seg - 1] + f * (poly[seg] - poly[seg - 1]);
        const Eigen::Vector2d dir = poly[seg] - poly[seg - 1];
        double yaw = std::atan2(dir.y(), dir.x());
        yaw += 2.0 * M_PI * std::round((yaw_prev - yaw) / (2.0 * M_PI));
        inner_.col(i - 1) << pt, yaw;
        yaw_prev = yaw;
    }
    tail_cur_(2, 0) += 2.0 * M_PI * std::round((yaw_prev - tail_cur_(2, 0)) / (2.0 * M_PI));

    // trapezoidal surge profile at 70% of the limits from the current speed
    const double v = 0.7 * lim_.max_vx, a = 0.7 * lim_.max_ax, v0 = std::min(head.col(1).head<2>().norm(), v);
    const double l_ramp = (v * v - v0 * v0) / (2 * a) + v * v / (2 * a);
    const double T = L >= l_ramp ? (v - v0) / a + v / a + (L - l_ramp) / v
                                 : 2.0 * std::sqrt(std::max(L, 1e-3) / a);
    times_ = Eigen::VectorXd::Constant(N_, std::max(T / N_, 0.1));
    return std::isfinite(T);
}

bool HoloPlanner::plan(const FlatTrajData &flat, const PVA &head)
{
    const ros::Time t_start = ros::Time::now();
    head_cur_ = head;
    tail_cur_.setZero();
    tail_cur_.col(0) = flat.final_state_XYTheta;

    // The optimizer always targets the configured clearance (no per-replan relaxation: relaxing to the start's
    // clearance on every 0.08 s replan ratcheted the ROV towards obstacles). If the start is already inside the
    // verification radius (an obstacle revealed late in unknown space), accept trajectories that never get closer
    // than the start is, so the ROV can plan its way out instead of being stuck.
    safe_dis_ = safe_dis_cfg_;
    const double start_dis = map_->getDistanceReal(head.col(0).head<2>());
    const double min_ok = std::min(final_min_safe_dis_, start_dis - 0.02);
    if (min_ok < final_min_safe_dis_)
        ROS_WARN("[holo] start is %.3f m from an obstacle (< %.3f): accepting an escape trajectory down to %.3f m",
                 start_dis, final_min_safe_dis_, min_ok);

    bool ok = false;
    double min_dis = 0.0;
    // The limits are soft penalties: verify the result and, if it is unsafe or exceeds a limit by more than
    // limit_tolerance, re-optimize with stronger weights. A trajectory that still fails is NOT published.
    // Attempt 0 starts from the initial guess, later attempts warm-start from the previous solution.
    std::string why;
    double excess = 0.0;
    const bool head_at_rest = head.col(1).norm() < 1e-6 && head.col(2).norm() < 1e-6;
    for (int attempt = 0; attempt < max_attempts_; attempt++)
    {
        if ((attempt == 0 && !initialGuess(flat, head)) || !optimize()) { why = "optimizer error"; break; }
        Trajectory<5, 3> traj;
        minco_.getTrajectory(traj);
        if (checkFinalCollision(min_dis, min_ok))
        {
            why = "too close to an obstacle";
            ROS_WARN("[holo] attempt %d: trajectory comes %.3f m from an obstacle (< %.3f), retrying", attempt, min_dis,
                     min_ok);
            wt_.time *= 0.75;
            wt_.collision *= 2.0;
            continue;
        }
        excess = limitExcess(traj, false);
        if (excess > limit_tol_ && attempt + 1 == max_attempts_ && head_at_rest && stretchTime(excess))
        {
            ok = true;  // last resort: same path, uniformly slower (start and goal at rest, so the boundary holds)
            break;
        }
        if (excess > limit_tol_)
        {
            why = "kinematic / thrust limits exceeded";
            ROS_WARN("[holo] attempt %d: a placeholder limit is exceeded by %.1f%% (> %.1f%%), retrying with 10x limit "
                     "weights", attempt, 100.0 * excess, 100.0 * limit_tol_);
            for (double *w : {&wt_.vel, &wt_.omega, &wt_.acc, &wt_.domega, &wt_.alloc}) *w *= 10.0;
            continue;
        }
        ok = true;
        break;
    }
    wt_ = wt_backup_;
    safe_dis_ = safe_dis_cfg_;
    if (!ok)
    {
        ROS_ERROR("[holo] no acceptable trajectory (%s)", why.c_str());
        return false;
    }
    minco_.getTrajectory(final_traj_);
    head_ = head_cur_;
    tail_ = tail_cur_;
    final_inner_ = inner_;
    final_times_ = times_;
    const double Tf = final_traj_.getTotalDuration();
    const Eigen::Vector3d pe = final_traj_.getPos(Tf) - tail_.col(0);
    ROS_INFO("[holo] plan ok: %d pieces, %.2f s, min obstacle distance %.3f m, back end %.1f ms, endpoint error "
             "pos %.2e m yaw %.2e rad vel %.2e (hard boundary condition)", N_, Tf, min_dis,
             (ros::Time::now() - t_start).toSec() * 1e3, pe.head<2>().norm(), std::fabs(pe(2)),
             final_traj_.getVel(Tf).norm());
    limitExcess(final_traj_, true);
    return true;
}

// Uniform time scaling of the current solution (k < 4): velocities scale with 1/k, accelerations with 1/k^2; the path
// (and so its clearance) is unchanged. Only valid when the head and tail velocity / acceleration are zero.
bool HoloPlanner::stretchTime(double excess)
{
    const Eigen::VectorXd t0 = times_;
    for (double k = 1.0 + 0.3 * std::min(excess, 1.0); k < 4.0; k *= 1.01)
    {
        times_ = t0 * k;
        minco_.setConditions(head_cur_, tail_cur_, N_, energy_w_);
        minco_.setParameters(inner_, times_);
        Trajectory<5, 3> traj;
        minco_.getTrajectory(traj);
        const double e = limitExcess(traj, false);
        if (e <= limit_tol_)
        {
            ROS_WARN("[holo] limits exceeded by %.1f%% after the last attempt: time-stretched x%.3f (same path), now %.2f%%",
                     100.0 * excess, k, 100.0 * e);
            return true;
        }
    }
    times_ = t0;
    minco_.setParameters(inner_, times_);
    return false;
}

bool HoloPlanner::optimize()
{
    minco_.setConditions(head_cur_, tail_cur_, N_, energy_w_);
    Eigen::VectorXd x(3 * (N_ - 1) + N_);
    Eigen::Map<Eigen::Matrix3Xd>(x.data(), 3, N_ - 1) = inner_;
    realT2Virtual(times_, x.segment(3 * (N_ - 1), N_));

    nh_.getParamCached(ros::this_node::getName() + "/holo/gradient_check", gradient_check_);  // settable at runtime
    if (gradient_check_) gradientCheck(x);   // every plan while enabled (debug only: ~10x slower)

    double cost = 0.0;
    iter_num_ = 0;
    const int result = lbfgs::lbfgs_optimize(x, cost, &HoloPlanner::costCallback, nullptr, nullptr, this, lbfgs_params_);
    if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
        ROS_WARN("[holo] L-BFGS: maximum line search reached, using the last iterate");
    else if (!(result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_CANCELED ||
               result == lbfgs::LBFGSERR_MAXIMUMITERATION))
    {
        ROS_ERROR("[holo] L-BFGS failed: %s", lbfgs::lbfgs_strerror(result));
        return false;
    }
    // Re-evaluate at the solution so minco_, inner_, times_ and the per-term costs match x
    Eigen::VectorXd g(x.size());
    cost = evaluate(x, g);
    if (!std::isfinite(cost)) return false;
    ROS_INFO("[holo] L-BFGS %d evals, cost %.3f (energy %.3f time %.3f vel %.3f acc %.3f alloc %.3f coll %.3f meanT %.3f)",
             iter_num_, cost, c_energy_, c_time_, c_vel_, c_acc_, c_alloc_, c_coll_, c_meanT_);
    return true;
}

// ------------------------------------------------------------------------------------------------ cost

double HoloPlanner::costCallback(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    return static_cast<HoloPlanner *>(ptr)->evaluate(x, g);
}

double HoloPlanner::evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    iter_num_++;
    if (!x.allFinite() || x.norm() > 1e5) { g.setZero(); return INFINITY; }
    const int nP = 3 * (N_ - 1);
    inner_ = Eigen::Map<const Eigen::Matrix3Xd>(x.data(), 3, N_ - 1);
    const Eigen::Map<const Eigen::VectorXd> tau(x.data() + nP, N_);
    virtualT2Real(tau, times_);

    minco_.setParameters(inner_, times_);
    double cost = 0.0;
    minco_.getEnergy(cost);
    c_energy_ = cost;
    minco_.getEnergyPartialGradByCoeffs(gdC_);
    minco_.getEnergyPartialGradByTimes(gdT_);

    attachPenalty(cost);

    minco_.propogateGrad(gdC_, gdT_, gradByPoints_, gradByTimes_);
    c_time_ = wt_.time * times_.sum();
    cost += c_time_;
    gradByTimes_.array() += wt_.time;

    g.resize(x.size());
    Eigen::Map<Eigen::Matrix3Xd>(g.data(), 3, N_ - 1) = gradByPoints_;
    backwardGradT(tau, gradByTimes_, g.segment(nP, N_));
    return cost;
}

void HoloPlanner::attachPenalty(double &cost)
{
    c_vel_ = c_acc_ = c_alloc_ = c_coll_ = c_meanT_ = 0.0;
    collision_points_.clear();
    const auto &C = minco_.getCoeffs();
    const double meanT = times_.mean();
    Eigen::Matrix<double, 6, 1> b0, b1, b2, b3;

    for (int i = 0; i < N_; i++)
    {
        const Eigen::Matrix<double, 6, 3> c = C.block<6, 3>(6 * i, 0);
        const double step = times_(i) / K_;
        for (int j = 0; j <= K_; j++)
        {
            const double s1 = j * step, s2 = s1 * s1, s3 = s2 * s1, s4 = s2 * s2, s5 = s4 * s1;
            b0 << 1.0, s1, s2, s3, s4, s5;
            b1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            b2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
            b3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
            const Eigen::Vector3d p = c.transpose() * b0, v = c.transpose() * b1, a = c.transpose() * b2,
                                  jr = c.transpose() * b3;
            const double omg = (j == 0 || j == K_) ? 0.5 : 1.0;
            const double alpha = double(j) / K_;

            const double ct = std::cos(p(2)), st = std::sin(p(2));
            const double vbx = ct * v(0) + st * v(1), vby = -st * v(0) + ct * v(1), w = v(2);
            const double abx = ct * a(0) + st * a(1), aby = -st * a(0) + ct * a(1), dw = a(2);
            // time derivative of the body velocity, d/dt (R^T pdot) = R^T pddot - omega S v_b: the quantity the NMPC
            // rate limits and the simulator's acceleration limits act on
            const double dvx = abx + w * vby, dvy = aby - w * vbx;
            const Eigen::Vector3d ex(ct, st, 0.0), ey(-st, ct, 0.0), ez(0.0, 0.0, 1.0);
            // the penalties target (1 - limit_margin) x each limit; verification uses the limit itself
            const double sm = 1.0 - limit_margin_;
            const double Lvx = sm * lim_.max_vx, Lnvx = sm * lim_.min_vx, Lvy = sm * lim_.max_vy,
                         Lw = sm * lim_.max_omega, Lax = sm * lim_.max_ax, Lay = sm * lim_.max_ay,
                         Ldw = sm * lim_.max_domega;

            double P = 0.0;  // weighted penalty at this sample
            Eigen::Vector3d gP = Eigen::Vector3d::Zero(), gV = Eigen::Vector3d::Zero(), gA = Eigen::Vector3d::Zero();
            double *term = nullptr;
            // q > 0 is a violation; dq/dp, dq/dv, dq/da given; weight wq
            auto add = [&](double q, const Eigen::Vector3d &dqp, const Eigen::Vector3d &dqv, const Eigen::Vector3d &dqa,
                           double wq) {
                if (q <= 0.0) return;
                double f, df;
                smoothL1(q, f, df);
                P += wq * f;
                *term += omg * step * wq * f;
                gP += wq * df * dqp;
                gV += wq * df * dqv;
                gA += wq * df * dqa;
            };
            const Eigen::Vector3d Z = Eigen::Vector3d::Zero();

            // independent body-frame velocity limits
            term = &c_vel_;
            add(vbx - Lvx, vby * ez, ex, Z, wt_.vel);
            add(Lnvx - vbx, -vby * ez, -ex, Z, wt_.vel);
            add(vby * vby - Lvy * Lvy, -2.0 * vby * vbx * ez, 2.0 * vby * ey, Z, wt_.vel);
            add(w * w - Lw * Lw, Z, 2.0 * w * ez, Z, wt_.omega);
            // body-velocity rates (vxdot, vydot, omegadot), same quantities and limits as holo_nmpc / the simulator
            term = &c_acc_;
            add(dvx * dvx - Lax * Lax, 2.0 * dvx * dvy * ez, 2.0 * dvx * (w * ey + vby * ez), 2.0 * dvx * ex, wt_.acc);
            add(dvy * dvy - Lay * Lay, -2.0 * dvy * dvx * ez, -2.0 * dvy * (w * ex + vbx * ez), 2.0 * dvy * ey, wt_.acc);
            add(dw * dw - Ldw * Ldw, Z, Z, 2.0 * dw * ez, wt_.domega);
            // thruster saturation / allocation reachability (placeholder M, D, thrust limits)
            if (alloc_enable_)
            {
                term = &c_alloc_;
                const Eigen::Vector3d tau(mass_ * abx + drag_(0) * vbx, mass_ * aby + drag_(1) * vby,
                                          inertia_z_ * dw + drag_(2) * w);
                // d tau / d theta, d tau / d v, d tau / d a  (columns = tau components)
                const Eigen::Vector3d dth(mass_ * aby + drag_(0) * vby, -mass_ * abx - drag_(1) * vbx, 0.0);
                for (size_t k = 0; k < facet_n_.size(); k++)
                {
                    const Eigen::Vector3d &n = facet_n_[k];
                    const double q = n.dot(tau) - facet_h_[k] + limit_margin_;
                    const Eigen::Vector3d dqv = n(0) * drag_(0) * ex + n(1) * drag_(1) * ey + n(2) * drag_(2) * ez;
                    const Eigen::Vector3d dqa = n(0) * mass_ * ex + n(1) * mass_ * ey + n(2) * inertia_z_ * ez;
                    add(q, n.dot(dth) * ez, dqv, dqa, wt_.alloc);
                }
            }
            // ESDF safety for every body check point (circle model: the single point (0, 0))
            term = &c_coll_;
            for (const auto &cp : check_points_)
            {
                const Eigen::Vector2d rcp(ct * cp.x() - st * cp.y(), st * cp.x() + ct * cp.y());
                const Eigen::Vector2d drcp(-st * cp.x() - ct * cp.y(), ct * cp.x() - st * cp.y());
                Eigen::Vector2d grad = Eigen::Vector2d::Zero();
                const double sdf = map_->getDistWithGradBilinear(p.head<2>() + rcp, grad, safe_dis_);
                const double q = safe_dis_ - sdf;
                if (q > 0.0) collision_points_.push_back(p.head<2>() + rcp);
                add(q, Eigen::Vector3d(-grad.x(), -grad.y(), -grad.dot(drcp)), Z, Z, wt_.collision);
            }

            if (P > 0.0)
            {
                cost += omg * step * P;
                gdC_.block<6, 3>(6 * i, 0) += omg * step * (b0 * gP.transpose() + b1 * gV.transpose() + b2 * gA.transpose());
                gdT_(i) += omg * (P / K_ + step * alpha * (gP.dot(v) + gV.dot(a) + gA.dot(jr)));
            }
        }

        // piece-duration balance (same as MSPlanner)
        for (const double bound : {mean_time_low_, mean_time_upp_})
        {
            const double d = times_(i) - meanT * bound;
            if ((bound == mean_time_low_ && d < 0.0) || (bound == mean_time_upp_ && d > 0.0))
            {
                cost += wt_.mean_time * d * d;
                c_meanT_ += wt_.mean_time * d * d;
                gdT_.array() += wt_.mean_time * 2.0 * d * (-bound / N_);
                gdT_(i) += wt_.mean_time * 2.0 * d;
            }
        }
    }
}

inline void HoloPlanner::smoothL1(const double &x, double &f, double &df) const
{
    const double pe = smooth_eps_, half = 0.5 * pe, f3c = 1.0 / (pe * pe), f4c = -0.5 * f3c / pe;
    if (x < pe)
    {
        f = (f4c * x + f3c) * x * x * x;
        df = (4.0 * f4c * x + 3.0 * f3c) * x * x;
    }
    else
    {
        f = x - half;
        df = 1.0;
    }
}

void HoloPlanner::realT2Virtual(const Eigen::VectorXd &RT, Eigen::Ref<Eigen::VectorXd> VT)
{
    for (int i = 0; i < RT.size(); ++i)
        VT(i) = RT(i) > 1.0 ? (std::sqrt(2.0 * RT(i) - 1.0) - 1.0) : (1.0 - std::sqrt(2.0 / RT(i) - 1.0));
}

void HoloPlanner::virtualT2Real(const Eigen::Ref<const Eigen::VectorXd> &VT, Eigen::VectorXd &RT)
{
    RT.resize(VT.size());
    for (int i = 0; i < VT.size(); ++i)
        RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
}

void HoloPlanner::backwardGradT(const Eigen::Ref<const Eigen::VectorXd> &tau, const Eigen::VectorXd &gradT,
                                Eigen::Ref<Eigen::VectorXd> gradTau)
{
    for (int i = 0; i < tau.size(); i++)
    {
        double d;
        if (tau(i) > 0) d = tau(i) + 1.0;
        else
        {
            const double den = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            d = (1.0 - tau(i)) / (den * den);
        }
        gradTau(i) = gradT(i) * d;
    }
}

void HoloPlanner::gradientCheck(const Eigen::VectorXd &x0)
{
    // Central finite differences vs the analytic gradient, first with all terms, then one penalty term at a time
    // (energy + time always on).
    const HoloWeights full = wt_;
    const char *names[] = {"all", "vel", "omega", "acc", "domega", "alloc", "collision", "mean_time", "none"};
    for (int mode = 0; mode < 9; mode++)
    {
        wt_ = full;
        if (mode > 0)
        {
            HoloWeights z = full;
            z.vel = z.omega = z.acc = z.domega = z.alloc = z.collision = z.mean_time = 0.0;
            double *src[] = {nullptr, &z.vel, &z.omega, &z.acc, &z.domega, &z.alloc, &z.collision, &z.mean_time};
            const double val[] = {0, full.vel, full.omega, full.acc, full.domega, full.alloc, full.collision, full.mean_time};
            if (mode < 8) *src[mode] = val[mode];
            wt_ = z;
        }
        Eigen::VectorXd g(x0.size()), gtmp(x0.size());
        const double f0 = evaluate(x0, g);
        double worst = 0.0, fdw = 0.0, gw = 0.0;
        int kw = -1;
        for (int k = 0; k < x0.size(); k++)
        {
            const double h = 1e-6 * std::max(1.0, std::fabs(x0(k)));
            Eigen::VectorXd xp = x0, xm = x0;
            xp(k) += h;
            xm(k) -= h;
            const double fd = (evaluate(xp, gtmp) - evaluate(xm, gtmp)) / (2.0 * h);
            const double rel = std::fabs(fd - g(k)) / std::max(1.0, std::fabs(fd) + std::fabs(g(k)));
            if (rel > worst) { worst = rel; kw = k; fdw = fd; gw = g(k); }
        }
        ROS_WARN("[holo] gradient check [%s] f=%.3f over %ld vars (%d waypoint coords, %d times): max rel error %.2e "
                 "(var %d: fd %.4f analytic %.4f)", names[mode], f0, long(x0.size()), 3 * (N_ - 1), N_, worst, kw, fdw, gw);
    }
    wt_ = full;
    Eigen::VectorXd g(x0.size());
    evaluate(x0, g);
}

// ------------------------------------------------------------------------------------------------ checks / output

bool HoloPlanner::checkFinalCollision(double &min_dis, double min_ok) const
{
    Trajectory<5, 3> traj;
    minco_.getTrajectory(traj);
    min_dis = 1e10;
    const double T = traj.getTotalDuration();
    const int n = std::max(2, final_check_num_ * N_ * 4);
    for (int k = 0; k <= n; k++)
    {
        const Eigen::Vector3d p = traj.getPos(T * k / n);
        const double ct = std::cos(p(2)), st = std::sin(p(2));
        for (const auto &cp : check_points_)
        {
            const Eigen::Vector2d q = p.head<2>() + Eigen::Vector2d(ct * cp.x() - st * cp.y(), st * cp.x() + ct * cp.y());
            min_dis = std::min(min_dis, map_->getDistWithGradBilinear(q));
        }
    }
    return min_dis < min_ok;
}

// Log the largest value of each constrained quantity on the final trajectory relative to its placeholder limit.
// Largest relative excess over the placeholder limits on traj (0.1 = 10% over; the allocation term is the excess as
// a fraction of the thrust range). log = also print the peaks.
double HoloPlanner::limitExcess(const Trajectory<5, 3> &traj, bool log) const
{
    double mvx = -1e9, nvx = 1e9, mvy = 0, mw = 0, max_ = 0, may = 0, mdw = 0, util = -1e9;
    const double T = traj.getTotalDuration();
    const int ns = std::max(2, int(std::ceil(T / 0.01)));
    for (int k = 0; k <= ns; k++)
    {
        const double t = T * k / ns;
        const Eigen::Vector3d p = traj.getPos(t), v = traj.getVel(t), a = traj.getAcc(t);
        const double ct = std::cos(p(2)), st = std::sin(p(2));
        const double vbx = ct * v(0) + st * v(1), vby = -st * v(0) + ct * v(1);
        const double abx = ct * a(0) + st * a(1), aby = -st * a(0) + ct * a(1);
        const double dvx = abx + v(2) * vby, dvy = aby - v(2) * vbx;
        mvx = std::max(mvx, vbx); nvx = std::min(nvx, vbx);
        mvy = std::max(mvy, std::fabs(vby)); mw = std::max(mw, std::fabs(v(2)));
        max_ = std::max(max_, std::fabs(dvx)); may = std::max(may, std::fabs(dvy)); mdw = std::max(mdw, std::fabs(a(2)));
        if (alloc_enable_)
        {
            const Eigen::Vector3d tau(mass_ * abx + drag_(0) * vbx, mass_ * aby + drag_(1) * vby,
                                      inertia_z_ * a(2) + drag_(2) * v(2));
            for (size_t k = 0; k < facet_n_.size(); k++) util = std::max(util, facet_n_[k].dot(tau) - facet_h_[k]);
        }
    }
    const double ex = std::max({mvx / lim_.max_vx, nvx / lim_.min_vx, mvy / lim_.max_vy, mw / lim_.max_omega,
                                max_ / lim_.max_ax, may / lim_.max_ay, mdw / lim_.max_domega}) - 1.0;
    const double excess = std::max(ex, alloc_enable_ ? util : -1.0);
    if (!log) return excess;
    ROS_INFO("[holo] peaks (placeholder limits): vx %.3f/%.2f [min %.3f/%.2f]  |vy| %.3f/%.2f  |w| %.3f/%.2f  "
             "|dvx| %.3f/%.2f  |dvy| %.3f/%.2f  |dw| %.3f/%.2f  alloc %.4f (<=0: inside the reserve)  max_ratio %.4f",
             mvx, lim_.max_vx, nvx, lim_.min_vx, mvy, lim_.max_vy, mw, lim_.max_omega, max_, lim_.max_ax, may,
             lim_.max_ay, mdw, lim_.max_domega, util, ex + 1.0);
    return excess;
}

// Smallest obstacle distance (centre + check points) of the current trajectory from time t to its end.
double HoloPlanner::remainingClearance(double t) const
{
    double d = 1e10;
    const double T = final_traj_.getTotalDuration();
    for (double s = std::max(t, 0.0); s <= T + 1e-9; s += 0.05)
    {
        const Eigen::Vector3d p = final_traj_.getPos(std::min(s, T));
        const double ct = std::cos(p(2)), st = std::sin(p(2));
        for (const auto &cp : check_points_)
            d = std::min(d, map_->getDistWithGradBilinear(
                                p.head<2>() + Eigen::Vector2d(ct * cp.x() - st * cp.y(), st * cp.x() + ct * cp.y())));
    }
    return d;
}

HoloPlanner::PVA HoloPlanner::getPVA(double t) const
{
    PVA s;
    t = std::min(std::max(t, 0.0), final_traj_.getTotalDuration());
    s.col(0) = final_traj_.getPos(t);
    s.col(1) = final_traj_.getVel(t);
    s.col(2) = final_traj_.getAcc(t);
    return s;
}

Eigen::Vector3d HoloPlanner::getBodyVel(double t) const
{
    const PVA s = getPVA(t);
    const double ct = std::cos(s(2, 0)), st = std::sin(s(2, 0));
    return Eigen::Vector3d(ct * s(0, 1) + st * s(1, 1), -st * s(0, 1) + ct * s(1, 1), s(2, 1));
}

std::vector<Eigen::Vector3d> HoloPlanner::getPath(double t0, double t1) const
{
    std::vector<Eigen::Vector3d> path;
    const double T = final_traj_.getTotalDuration();
    t0 = std::min(std::max(t0, 0.0), T);
    t1 = std::min(std::max(t1, t0), T);
    for (double t = t0; t < t1; t += predict_res_) path.push_back(final_traj_.getPos(t));
    path.push_back(final_traj_.getPos(t1));
    return path;
}

void HoloPlanner::pathPub(const ros::Publisher &pub) const
{
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    const double T = final_traj_.getTotalDuration();
    for (double t = 0.0; t <= T + 1e-9; t += 0.02)
    {
        const Eigen::Vector3d p = final_traj_.getPos(std::min(t, T));
        geometry_msgs::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x();
        ps.pose.position.y = p.y();
        ps.pose.position.z = 0.15;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(p.z());
        path.poses.push_back(ps);
    }
    pub.publish(path);
}

void HoloPlanner::pointPub(const ros::Publisher &pub, const Eigen::Vector3d &color) const
{
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = "Innerpoint";
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::DELETEALL;
    pub.publish(m);
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.045;
    m.color.a = 1.0;
    m.color.r = color.x() / 255.0;
    m.color.g = color.y() / 255.0;
    m.color.b = color.z() / 255.0;
    for (int i = 0; i < final_traj_.getPieceNum(); i++)
    {
        const Eigen::Vector3d p = final_traj_.getJuncPos(i + 1);
        geometry_msgs::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.15;
        m.points.push_back(pt);
    }
    pub.publish(m);
}
