// Thruster allocation for the ROV V4 (8 x T500): maps the holonomic NMPC command to per-thruster thrust.
//
//   1. desired body wrench (PLACEHOLDER rigid-body + linear-drag model; no added mass or quadratic drag):
//          X = m (vxdot - r vy) + d_u vx,   Y = m (vydot + r vx) + d_v vy,   N = Iz rdot + d_r r
//      (m (vdot_b + r S v_b) = m R^T pddot, the same wrench the planner's reachability constraint uses)
//   2. QP (OSQP), heave / roll / pitch as HARD equalities, the planar wrench in least squares:
//          min_f  1/2 (B_p f - tau_p)^T W_p (B_p f - tau_p) + 1/2 eps |f|^2
//          s.t.   B_z f = 0,  B_K f = 0,  B_M f = 0      (Z, roll, pitch)
//                 f_min <= f_i <= f_max                  (all 8 thrusters)
//      B (6 x 8) from thruster_layout.csv: column i = [d_i ; r_i x d_i]; rows p = (X, Y, N). Moments are about the
//      hull-envelope centre (centre of mass unknown -> TODO); Z = 0 assumes neutral buoyancy (unknown -> TODO).
//      The horizontal thrusters (z = 0.045 m) create roll / pitch moments that the vertical thrusters cancel;
//      check_allocation.py shows this needs at most ~5.5 N per vertical thruster for ANY horizontal command, so
//      the equalities are always feasible with the placeholder limits.
//   Optional (Gazebo, rov_gazebo_bridge):
//     - zkm_target/enable: the heave / roll / pitch equalities take the targets on ~zkm (WrenchStamped force.z,
//       torque.x, torque.y; from the bridge's depth / attitude hold) instead of 0.
//     - velocity_feedback/enable: tau_p += K (v_cmd - v_meas) with v_meas the body twist on ~odom, closing the
//       velocity loop around the feed-forward model (the 2D simulator follows the command exactly and needs none).
//   thrusters/moment_ref: point the moments are taken about (estimated centre of mass; default the hull centre).
//   3. If the equality-constrained QP is infeasible anyway (e.g. once real buoyancy / CoM offsets are added), it falls
//      back to weighted least squares over all 6 axes (`weights`) and reports `saturated`.
//   4. publish thrusts, desired / achieved wrench and `saturated` (= planar residual |B_p f - tau_p| above
//      saturation_tolerance, or the fallback was used).
//
// The kinematic simulator does NOT consume the thrusts: this layer shows what the thrusters would have to do and
// whether the request is reachable under the placeholder limits. It does not prove dynamic feasibility.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

class ThrusterAllocation
{
public:
    explicit ThrusterAllocation(ros::NodeHandle &pnh) : pnh_(pnh)
    {
        std::vector<double> x, y, z, ax, ay, az, dr, w;
        pnh_.getParam("thrusters/names", names_);
        pnh_.getParam("thrusters/x", x);
        pnh_.getParam("thrusters/y", y);
        pnh_.getParam("thrusters/z", z);
        pnh_.getParam("thrusters/axis_x", ax);
        pnh_.getParam("thrusters/axis_y", ay);
        pnh_.getParam("thrusters/axis_z", az);
        pnh_.param("thrusters/f_max", fmax_, 40.0);
        pnh_.param("thrusters/f_min", fmin_, -30.0);
        // inertial feed-forward uses the EFFECTIVE values (dry + added mass), scaled by feedforward_scale
        double m_plain, iz_plain;
        pnh_.param("holo/alloc/mass", m_plain, 60.0);
        pnh_.param("holo/alloc/inertia_z", iz_plain, 6.0);
        pnh_.param("holo/alloc/mass_effective", mass_, m_plain);
        pnh_.param("holo/alloc/inertia_z_effective", iz_, iz_plain);
        pnh_.param("feedforward_scale", ff_scale_, 1.0);
        pnh_.param("holo/alloc/linear_drag", dr, std::vector<double>{60.0, 90.0, 15.0});
        pnh_.param("weights", w, std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        pnh_.param("regularisation", eps_, 1e-4);
        pnh_.param("saturation_tolerance", tol_, 0.5);
        std::vector<double> mref, kv;
        pnh_.param("thrusters/moment_ref", mref, std::vector<double>{0.0, 0.0, 0.0});
        pnh_.param("zkm_target/enable", zkm_enable_, false);
        pnh_.param("zkm_target/timeout", zkm_timeout_, 0.5);
        pnh_.param("velocity_feedback/enable", vfb_enable_, false);
        pnh_.param("velocity_feedback/gains", kv, std::vector<double>{150.0, 150.0, 40.0});
        pnh_.param("velocity_feedback/timeout", odom_timeout_, 0.5);
        Kv_ = Eigen::Vector3d(kv.at(0), kv.at(1), kv.at(2));
        mref_ = Eigen::Vector3d(mref.at(0), mref.at(1), mref.at(2));
        n_ = x.size();
        if (n_ == 0 || y.size() != n_ || z.size() != n_ || ax.size() != n_ || ay.size() != n_ || az.size() != n_ ||
            names_.size() != n_)
        {
            ROS_FATAL("[allocation] thruster layout parameters missing or inconsistent");
            ros::shutdown();
            return;
        }
        drag_ = Eigen::Vector3d(dr.at(0), dr.at(1), dr.at(2));
        W_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(w.data());
        B_.resize(6, n_);
        for (size_t i = 0; i < n_; i++)
        {
            const Eigen::Vector3d r = Eigen::Vector3d(x[i], y[i], z[i]) - mref_;
            const Eigen::Vector3d d = Eigen::Vector3d(ax[i], ay[i], az[i]).normalized();
            B_.col(i) << d, r.cross(d);
        }
        Eigen::FullPivLU<Eigen::MatrixXd> lu(B_);
        ROS_WARN("[allocation] %zu thrusters, B rank %ld, thrust [%.1f, %.1f] N, m %.1f kg, Iz %.1f, drag (%.1f %.1f %.1f) "
                 "-- ALL PLACEHOLDERS except the layout (feed-forward x%.2f)", n_, long(lu.rank()), fmin_, fmax_, mass_,
                 iz_, drag_(0), drag_(1), drag_(2), ff_scale_);

        cmd_sub_ = pnh_.subscribe("cmd", 1, &ThrusterAllocation::cmdCallback, this);
        acc_sub_ = pnh_.subscribe("accel", 1, &ThrusterAllocation::accCallback, this);
        if (zkm_enable_)
        {
            zkm_sub_ = pnh_.subscribe("zkm", 1, &ThrusterAllocation::zkmCallback, this);
            pnh_.param("cmd_timeout", cmd_timeout_, 0.2);
            timer_ = pnh_.createTimer(ros::Duration(0.02), &ThrusterAllocation::timerCallback, this);
        }
        if (vfb_enable_) odom_sub_ = pnh_.subscribe("odom", 1, &ThrusterAllocation::odomCallback, this);
        ROS_WARN("[allocation] moments about (%.3f, %.3f, %.3f); heave/roll/pitch targets: %s; velocity feedback: %s "
                 "(K %.0f %.0f %.0f)", mref_(0), mref_(1), mref_(2), zkm_enable_ ? "~zkm" : "0",
                 vfb_enable_ ? "ON" : "off", Kv_(0), Kv_(1), Kv_(2));
        thrust_pub_ = pnh_.advertise<std_msgs::Float64MultiArray>("thrust", 1);
        wdes_pub_ = pnh_.advertise<geometry_msgs::WrenchStamped>("wrench_desired", 1);
        wach_pub_ = pnh_.advertise<geometry_msgs::WrenchStamped>("wrench_achieved", 1);
        sat_pub_ = pnh_.advertise<std_msgs::Bool>("saturated", 1);
    }

private:
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_, acc_sub_, zkm_sub_, odom_sub_;
    bool zkm_enable_ = false, vfb_enable_ = false;
    double zkm_timeout_ = 0.5, odom_timeout_ = 0.5;
    Eigen::Vector3d zkm_ = Eigen::Vector3d::Zero(), vmeas_ = Eigen::Vector3d::Zero(), Kv_, mref_;
    ros::Time zkm_time_, odom_time_, cmd_time_;
    Eigen::Vector3d cmd_ = Eigen::Vector3d::Zero();
    double cmd_timeout_ = 0.2;
    ros::Timer timer_;
    ros::Publisher thrust_pub_, wdes_pub_, wach_pub_, sat_pub_;
    std::vector<std::string> names_;
    size_t n_ = 0;
    double fmax_, fmin_, mass_, iz_, eps_, tol_, ff_scale_ = 1.0;
    Eigen::Vector3d drag_, acc_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 1> W_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd f_prev_;
    long n_calls_ = 0, n_sat_ = 0;

    void zkmCallback(const geometry_msgs::WrenchStamped::ConstPtr &m)
    {
        zkm_ << m->wrench.force.z, m->wrench.torque.x, m->wrench.torque.y;
        zkm_time_ = ros::Time::now();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &m)
    {
        vmeas_ << m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.angular.z;
        odom_time_ = ros::Time::now();
    }

    void accCallback(const geometry_msgs::AccelStamped::ConstPtr &m)
    {
        acc_ << m->accel.linear.x, m->accel.linear.y, m->accel.angular.z;
    }

    // min 1/2 f^T P f + q^T f  s.t.  A f in [lo, up];  returns false if OSQP does not report a solution
    bool solveQp(const Eigen::MatrixXd &P, const Eigen::VectorXd &q, const Eigen::MatrixXd &A, const Eigen::VectorXd &lo,
                 const Eigen::VectorXd &up, Eigen::VectorXd &f) const
    {
        Eigen::SparseMatrix<double> Ps = P.sparseView(), As = A.sparseView();
        Eigen::VectorXd qq = q, l = lo, u = up;
        OsqpEigen::Solver s;
        s.settings()->setVerbosity(false);
        s.settings()->setMaxIteration(4000);
        s.settings()->setAbsoluteTolerance(1e-7);
        s.settings()->setRelativeTolerance(1e-7);
        s.settings()->setPolish(true);
        s.data()->setNumberOfVariables(n_);
        s.data()->setNumberOfConstraints(A.rows());
        if (!(s.data()->setHessianMatrix(Ps) && s.data()->setGradient(qq) && s.data()->setLinearConstraintsMatrix(As) &&
              s.data()->setLowerBound(l) && s.data()->setUpperBound(u) && s.initSolver() &&
              s.solveProblem() == OsqpEigen::ErrorExitFlag::NoError))
            return false;
        const auto st = s.getStatus();
        if (st != OsqpEigen::Status::Solved && st != OsqpEigen::Status::SolvedInaccurate) return false;
        f = s.getSolution();
        return f.allFinite();
    }

    // With heave / roll / pitch targets (Gazebo) the allocation must run even when no planar command arrives
    // (before the first trajectory, after an emergency stop): a 50 Hz timer allocates the last command, or zero
    // planar motion once it is older than cmd_timeout. Without targets (2D) it runs once per command, as before.
    void cmdCallback(const geometry_msgs::Twist::ConstPtr &m)
    {
        cmd_ << m->linear.x, m->linear.y, m->angular.z;
        cmd_time_ = ros::Time::now();
        if (!zkm_enable_) allocate(cmd_);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        const bool fresh = (ros::Time::now() - cmd_time_).toSec() < cmd_timeout_;
        if (!fresh) acc_.setZero();
        allocate(fresh ? cmd_ : Eigen::Vector3d::Zero());
    }

    void allocate(const Eigen::Vector3d &v)
    {
        Eigen::Matrix<double, 6, 1> tau;
        const double ms = ff_scale_ * mass_, is = ff_scale_ * iz_;
        tau << ms * (acc_(0) - v(2) * v(1)) + drag_(0) * v(0), ms * (acc_(1) + v(2) * v(0)) + drag_(1) * v(1),
            0.0, 0.0, 0.0, is * acc_(2) + drag_(2) * v(2);
        const ros::Time now = ros::Time::now();
        if (vfb_enable_)
        {
            if ((now - odom_time_).toSec() < odom_timeout_)
            {
                const Eigen::Vector3d fb = Kv_.cwiseProduct(v - vmeas_);
                tau(0) += fb(0);
                tau(1) += fb(1);
                tau(5) += fb(2);
            }
            else
                ROS_WARN_THROTTLE(1.0, "[allocation] no odometry: velocity feedback off");
        }
        if (zkm_enable_)
        {
            if ((now - zkm_time_).toSec() < zkm_timeout_)
                tau.segment<3>(2) = zkm_;
            else
                ROS_WARN_THROTTLE(1.0, "[allocation] no heave/roll/pitch target on ~zkm: holding 0");
        }

        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n_);
        // 1) hard Z / K / M = 0, planar least squares
        Eigen::Matrix<double, 3, Eigen::Dynamic> Bp(3, n_), Bz(3, n_);
        Bp << B_.row(0), B_.row(1), B_.row(5);
        Bz << B_.row(2), B_.row(3), B_.row(4);
        const Eigen::Vector3d tp(tau(0), tau(1), tau(5)), wp(W_(0), W_(1), W_(5));
        Eigen::MatrixXd A(n_ + 3, n_);
        A << I, Bz;
        Eigen::VectorXd lo(n_ + 3), up(n_ + 3);
        lo << Eigen::VectorXd::Constant(n_, fmin_), tau.segment<3>(2);
        up << Eigen::VectorXd::Constant(n_, fmax_), tau.segment<3>(2);
        bool held = solveQp(Bp.transpose() * wp.asDiagonal() * Bp + eps_ * I, -Bp.transpose() * wp.cwiseProduct(tp), A,
                            lo, up, f);
        bool ok = held;
        // 2) fallback: weighted least squares over all 6 axes
        if (!held)
        {
            ok = solveQp(B_.transpose() * W_.asDiagonal() * B_ + eps_ * I, -B_.transpose() * W_.cwiseProduct(tau), I,
                         Eigen::VectorXd::Constant(n_, fmin_), Eigen::VectorXd::Constant(n_, fmax_), f);
            if (!ok) f.setZero();
            ROS_WARN_THROTTLE(1.0, "[allocation] heave/roll/pitch targets not feasible%s",
                              ok ? ", using weighted least squares" : " and fallback QP failed: zero thrust");
        }
        f = f.cwiseMax(fmin_).cwiseMin(fmax_);

        const Eigen::Matrix<double, 6, 1> ach = B_ * f;
        const double err = (Bp * f - tp).norm();
        const bool sat = !held || err > tol_;
        n_calls_++;
        if (sat) n_sat_++;
        if (sat)
            ROS_WARN_THROTTLE(2.0, "[allocation] request not reachable: |B_p f - tau_p| = %.2f (tau X %.1f Y %.1f N %.1f); "
                                   "%.1f%% of commands so far", err, tau(0), tau(1), tau(5), 100.0 * n_sat_ / n_calls_);

        std_msgs::Float64MultiArray th;
        th.data.assign(f.data(), f.data() + n_);
        thrust_pub_.publish(th);
        wdes_pub_.publish(wrench(tau));
        wach_pub_.publish(wrench(ach));
        std_msgs::Bool b;
        b.data = sat;
        sat_pub_.publish(b);
    }

    static geometry_msgs::WrenchStamped wrench(const Eigen::Matrix<double, 6, 1> &t)
    {
        geometry_msgs::WrenchStamped w;
        w.header.stamp = ros::Time::now();
        w.header.frame_id = "base_link";
        w.wrench.force.x = t(0);
        w.wrench.force.y = t(1);
        w.wrench.force.z = t(2);
        w.wrench.torque.x = t(3);
        w.wrench.torque.y = t(4);
        w.wrench.torque.z = t(5);
        return w;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thruster_allocation");
    ros::NodeHandle pnh("~");
    ThrusterAllocation alloc(pnh);
    ros::spin();
    return 0;
}
