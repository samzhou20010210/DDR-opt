// Planar holonomic NMPC for the ROV (robot:=rov_holo).
//
// Model (body frame b: x forward, y port; world frame: x, y, theta):
//     xdot = vx cos(theta) - vy sin(theta),  ydot = vx sin(theta) + vy cos(theta),  thetadot = omega
// State (x, y, theta), input u = (vx, vy, omega), forward-Euler with step dt over a horizon of N steps.
//
// Solver: real-time iteration (one Gauss-Newton SQP step per control cycle, the scheme ACADO's generated NMPC uses).
// Each cycle the previous optimal input sequence (shifted by the elapsed time) is rolled out through the nonlinear
// model from the measured state; the dynamics are linearised along that rollout, the input sequence is condensed
// into a QP and solved with OSQP. ACADO is not installed on this machine, so the car's ACADO NMPC (nmpc_controller)
// could not be regenerated; it stays unchanged for robot:=car.
//
// Constraints: independent input boxes min_vx <= vx <= max_vx, |vy| <= max_vy, |omega| <= max_omega and per-axis
// rate limits |u_k - u_(k-1)| <= (max_ax, max_ay, max_domega) * dt (u_(-1) = measured body velocity).
// ALL limits are PLACEHOLDERS (robot_rov_holo.yaml) until mass, drag and T500 thrust data exist.
//
// Topics: ~traj (carstatemsgs/Polynome, model HOLONOMIC), ~odom (nav_msgs/Odometry, body twist used if present),
// /planner/emergency_stop. Out: ~cmd (geometry_msgs/Twist body vx, vy, omega), ~accel (AccelStamped, body
// acceleration of the planned input, for the allocation feed-forward), ~ref (Twist, trajectory body velocity),
// ~ref_pose (PoseStamped, trajectory pose now), ~predicted_path (nav_msgs/Path), ~stats (Float64MultiArray [cycle_ms, osqp_solve_ms, osqp_iterations, ok,
// timer_period_ms]; one RTI step = one QP per control cycle).

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/AccelStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <chrono>
#include <cmath>
#include <memory>

#include "carstatemsgs/Polynome.h"
#include "gcopter/minco_holo.hpp"

class HoloNmpc
{
public:
    HoloNmpc(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
    {
        pnh_.param("dt", dt_, 0.1);
        pnh_.param("horizon", N_, 20);
        pnh_.param("rate", rate_, 50.0);
        std::vector<double> q, qf, r, rd;
        pnh_.param("Q", q, std::vector<double>{20.0, 20.0, 10.0});
        pnh_.param("Qf", qf, std::vector<double>{60.0, 60.0, 30.0});
        pnh_.param("R", r, std::vector<double>{0.5, 0.5, 0.5});
        pnh_.param("Rd", rd, std::vector<double>{5.0, 5.0, 5.0});
        Q_ = Eigen::Vector3d(q.at(0), q.at(1), q.at(2));
        Qf_ = Eigen::Vector3d(qf.at(0), qf.at(1), qf.at(2));
        R_ = Eigen::Vector3d(r.at(0), r.at(1), r.at(2));
        Rd_ = Eigen::Vector3d(rd.at(0), rd.at(1), rd.at(2));

        pnh_.param("holo/max_vx", umax_(0), 1.0);
        pnh_.param("holo/min_vx", umin_(0), -0.3);
        pnh_.param("holo/max_vy", umax_(1), 0.5);
        pnh_.param("holo/max_omega", umax_(2), 0.8);
        umin_(1) = -umax_(1);
        umin_(2) = -umax_(2);
        pnh_.param("holo/max_ax", alim_(0), 0.4);
        pnh_.param("holo/max_ay", alim_(1), 0.3);
        pnh_.param("holo/max_domega", alim_(2), 1.0);

        traj_sub_ = pnh_.subscribe("traj", 1, &HoloNmpc::trajCallback, this);
        odom_sub_ = pnh_.subscribe("odom", 1, &HoloNmpc::odomCallback, this);
        stop_sub_ = nh_.subscribe("/planner/emergency_stop", 1, &HoloNmpc::stopCallback, this);
        cmd_pub_ = pnh_.advertise<geometry_msgs::Twist>("cmd", 1);
        accel_pub_ = pnh_.advertise<geometry_msgs::AccelStamped>("accel", 1);
        ref_pub_ = pnh_.advertise<geometry_msgs::Twist>("ref", 1);
        ref_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("ref_pose", 10);
        path_pub_ = pnh_.advertise<nav_msgs::Path>("predicted_path", 1);
        stats_pub_ = pnh_.advertise<std_msgs::Float64MultiArray>("stats", 10);
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &HoloNmpc::control, this);

        ROS_WARN("[holo_nmpc] RTI-NMPC dt %.2f N %d rate %.0f Hz, u in [%.2f..%.2f, +-%.2f, +-%.2f], rate limits "
                 "(%.2f, %.2f, %.2f) (PLACEHOLDERS)", dt_, N_, rate_, umin_(0), umax_(0), umax_(1), umax_(2),
                 alim_(0), alim_(1), alim_(2));
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber traj_sub_, odom_sub_, stop_sub_;
    ros::Publisher cmd_pub_, accel_pub_, ref_pub_, ref_pose_pub_, path_pub_, stats_pub_;
    ros::Timer timer_;

    double dt_, rate_;
    int N_;
    Eigen::Vector3d Q_, Qf_, R_, Rd_, umin_, umax_, alim_;

    // trajectory (current and pending; a replanned trajectory starts at its traj_start_time)
    bool has_traj_ = false, has_pending_ = false;
    Trajectory<5, 3> traj_, pending_;
    double traj_start_ = 0.0, pending_start_ = 0.0;

    bool has_odom_ = false, has_twist_ = false, stopped_ = false;
    Eigen::Vector3d x_meas_, v_meas_;
    Eigen::Vector3d u_last_ = Eigen::Vector3d::Zero();

    Eigen::MatrixXd U_prev_;   // 3 x N, previous optimal inputs
    double U_prev_time_ = -1.0;

    void trajCallback(const carstatemsgs::Polynome::ConstPtr &msg)
    {
        if (msg->model != carstatemsgs::Polynome::HOLONOMIC)
        {
            ROS_ERROR_THROTTLE(1.0, "[holo_nmpc] received a non-holonomic trajectory; ignoring");
            return;
        }
        minco::MINCO_S3NU_ND<3>::PVA head, tail;
        auto col = [](const geometry_msgs::Vector3 &v) { return Eigen::Vector3d(v.x, v.y, v.z); };
        head << col(msg->init_p), col(msg->init_v), col(msg->init_a);
        tail << col(msg->tail_p), col(msg->tail_v), col(msg->tail_a);
        const int n = msg->t_pts.size();
        if (n < 1 || int(msg->innerpoints.size()) != n - 1) return;
        Eigen::Matrix3Xd inner(3, n - 1);
        for (int i = 0; i < n - 1; i++) inner.col(i) = col(msg->innerpoints[i]);
        Eigen::VectorXd T(n);
        for (int i = 0; i < n; i++) T(i) = msg->t_pts[i];
        minco::MINCO_S3NU_ND<3> m;
        m.setConditions(head, tail, n, Eigen::Vector3d::Ones());
        m.setParameters(inner, T);
        m.getTrajectory(pending_);
        pending_start_ = msg->traj_start_time.toSec();
        has_pending_ = true;
        stopped_ = false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        x_meas_ << msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation);
        if (has_odom_)  // keep yaw continuous with the previous measurement
            x_meas_(2) = x_meas_(2) + 2.0 * M_PI * std::round((xprev_yaw_ - x_meas_(2)) / (2.0 * M_PI));
        xprev_yaw_ = x_meas_(2);
        has_twist_ = !msg->child_frame_id.empty();
        if (has_twist_) v_meas_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
        has_odom_ = true;
    }
    double xprev_yaw_ = 0.0;

    void stopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (!msg->data) return;
        stopped_ = true;
        has_traj_ = has_pending_ = false;
        publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        ROS_WARN("[holo_nmpc] emergency stop");
    }

    // trajectory pose and body velocity at time t (clamped to the trajectory)
    void reference(double t, Eigen::Vector3d &pose, Eigen::Vector3d &ub) const
    {
        const double T = traj_.getTotalDuration();
        t = std::min(std::max(t, 0.0), T);
        pose = traj_.getPos(t);
        const Eigen::Vector3d v = traj_.getVel(t);
        const double c = std::cos(pose(2)), s = std::sin(pose(2));
        ub << c * v(0) + s * v(1), -s * v(0) + c * v(1), v(2);
    }

    void publishCmd(const Eigen::Vector3d &u, const Eigen::Vector3d &acc)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = u(0);
        cmd.linear.y = u(1);
        cmd.angular.z = u(2);
        cmd_pub_.publish(cmd);
        geometry_msgs::AccelStamped a;
        a.header.stamp = ros::Time::now();
        a.header.frame_id = "base_link";
        a.accel.linear.x = acc(0);
        a.accel.linear.y = acc(1);
        a.accel.angular.z = acc(2);
        accel_pub_.publish(a);
        u_last_ = u;
    }

    void control(const ros::TimerEvent &ev)
    {
        const auto wall0 = std::chrono::steady_clock::now();
        const double period_ms = ev.last_real.isZero() ? 0.0 : (ev.current_real - ev.last_real).toSec() * 1e3;
        const double now = ros::Time::now().toSec();
        if (has_pending_ && now >= pending_start_)
        {
            traj_ = pending_;
            traj_start_ = pending_start_;
            has_traj_ = true;
            has_pending_ = false;
        }
        if (!has_odom_ || !has_traj_ || stopped_) return;

        const int N = N_, nU = 3 * N;
        const double t0 = now - traj_start_;

        // nominal inputs: previous solution shifted by the elapsed time, else the trajectory's body velocity
        Eigen::MatrixXd Ub(3, N), Ur(3, N), Xr(3, N);
        for (int k = 0; k < N; k++)
        {
            Eigen::Vector3d pose, ub;
            reference(t0 + k * dt_, pose, ub);
            Ur.col(k) = ub;
            reference(t0 + (k + 1) * dt_, pose, ub);
            Xr.col(k) = pose;
            if (U_prev_time_ > 0.0)
            {
                const double tau = (now - U_prev_time_) / dt_ + k;
                const int i0 = std::min(int(std::floor(tau)), N - 1), i1 = std::min(i0 + 1, N - 1);
                const double f = std::min(std::max(tau - i0, 0.0), 1.0);
                Ub.col(k) = (1.0 - f) * U_prev_.col(std::max(i0, 0)) + f * U_prev_.col(i1);
            }
            else
                Ub.col(k) = Ur.col(k);
            Ub.col(k) = Ub.col(k).cwiseMax(umin_).cwiseMin(umax_);
        }

        // rollout and linearisation: x_(k+1) = x_k + dt G(theta_k) u_k
        std::vector<Eigen::Matrix3d> A(N), B(N);
        Eigen::MatrixXd Xb(3, N + 1);
        Xb.col(0) = x_meas_;
        for (int k = 0; k < N; k++)
        {
            const double th = Xb(2, k), c = std::cos(th), s = std::sin(th);
            const Eigen::Vector3d u = Ub.col(k);
            Eigen::Matrix3d G;
            G << c, -s, 0, s, c, 0, 0, 0, 1;
            Xb.col(k + 1) = Xb.col(k) + dt_ * G * u;
            A[k].setIdentity();
            A[k](0, 2) = dt_ * (-s * u(0) - c * u(1));
            A[k](1, 2) = dt_ * (c * u(0) - s * u(1));
            B[k] = dt_ * G;
        }
        // reference yaw unwrapped next to the nominal
        for (int k = 0; k < N; k++)
            Xr(2, k) += 2.0 * M_PI * std::round((Xb(2, k + 1) - Xr(2, k)) / (2.0 * M_PI));

        // condensed prediction: dX = S dU   (dX stacks x_1..x_N)
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nU, nU);
        for (int j = 0; j < N; j++)
        {
            Eigen::Matrix3d M = B[j];
            for (int k = j; k < N; k++)
            {
                S.block<3, 3>(3 * k, 3 * j) = M;
                if (k + 1 < N) M = A[k + 1] * M;
            }
        }
        Eigen::VectorXd Qb(nU), Rb(nU), Rdb(nU), xerr(nU), uerr(nU), Ubv(nU);
        for (int k = 0; k < N; k++)
        {
            Qb.segment<3>(3 * k) = (k == N - 1) ? Qf_ : Q_;
            Rb.segment<3>(3 * k) = R_;
            Rdb.segment<3>(3 * k) = Rd_;
            xerr.segment<3>(3 * k) = Xb.col(k + 1) - Xr.col(k);
            uerr.segment<3>(3 * k) = Ub.col(k) - Ur.col(k);
            Ubv.segment<3>(3 * k) = Ub.col(k);
        }
        // input differences D U - e, u_(-1) = measured body velocity (clamped into the box)
        const Eigen::Vector3d u_prev = (has_twist_ ? v_meas_ : u_last_).cwiseMax(umin_).cwiseMin(umax_);
        Eigen::MatrixXd D = Eigen::MatrixXd::Identity(nU, nU);
        for (int k = 1; k < N; k++) D.block<3, 3>(3 * k, 3 * (k - 1)) = -Eigen::Matrix3d::Identity();
        Eigen::VectorXd e = Eigen::VectorXd::Zero(nU);
        e.head<3>() = u_prev;
        const Eigen::VectorXd dU0 = D * Ubv - e;

        const Eigen::MatrixXd H = S.transpose() * Qb.asDiagonal() * S + Eigen::MatrixXd(Rb.asDiagonal()) +
                                  D.transpose() * Rdb.asDiagonal() * D;
        const Eigen::VectorXd g = S.transpose() * Qb.cwiseProduct(xerr) + Rb.cwiseProduct(uerr) +
                                  D.transpose() * Rdb.cwiseProduct(dU0);

        // constraints on dU: [I; D] dU in [l, u]
        Eigen::MatrixXd Ac(2 * nU, nU);
        Ac << Eigen::MatrixXd::Identity(nU, nU), D;
        Eigen::VectorXd lo(2 * nU), up(2 * nU);
        for (int k = 0; k < N; k++)
        {
            lo.segment<3>(3 * k) = umin_ - Ub.col(k);
            up.segment<3>(3 * k) = umax_ - Ub.col(k);
            lo.segment<3>(nU + 3 * k) = -alim_ * dt_ - dU0.segment<3>(3 * k);
            up.segment<3>(nU + 3 * k) = alim_ * dt_ - dU0.segment<3>(3 * k);
        }

        Eigen::SparseMatrix<double> Hs = H.sparseView(), As = Ac.sparseView();
        Eigen::VectorXd gv = g;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.settings()->setMaxIteration(4000);
        solver.settings()->setAbsoluteTolerance(1e-5);
        solver.settings()->setRelativeTolerance(1e-5);
        solver.data()->setNumberOfVariables(nU);
        solver.data()->setNumberOfConstraints(2 * nU);
        Eigen::VectorXd U = Ubv;
        bool ok = solver.data()->setHessianMatrix(Hs) && solver.data()->setGradient(gv) &&
                  solver.data()->setLinearConstraintsMatrix(As) && solver.data()->setLowerBound(lo) &&
                  solver.data()->setUpperBound(up) && solver.initSolver();
        const auto wall_s = std::chrono::steady_clock::now();
        ok = ok && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
        const double solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_s).count();
        const double iters = (solver.workspace() && solver.workspace()->info) ? solver.workspace()->info->iter : -1.0;
        ok = ok && (solver.getStatus() == OsqpEigen::Status::Solved ||
                    solver.getStatus() == OsqpEigen::Status::SolvedInaccurate);
        if (ok)
        {
            const Eigen::VectorXd dU = solver.getSolution();
            ok = dU.allFinite();
            if (ok) U = Ubv + dU;
        }
        if (!ok) ROS_WARN_THROTTLE(1.0, "[holo_nmpc] QP failed, applying the nominal input");

        U_prev_ = Eigen::Map<const Eigen::MatrixXd>(U.data(), 3, N);
        U_prev_time_ = now;
        const Eigen::Vector3d u0 = U_prev_.col(0).cwiseMax(umin_).cwiseMin(umax_);
        publishCmd(u0, (U_prev_.col(1) - U_prev_.col(0)) / dt_);

        geometry_msgs::Twist ref;
        ref.linear.x = Ur(0, 0);
        ref.linear.y = Ur(1, 0);
        ref.angular.z = Ur(2, 0);
        ref_pub_.publish(ref);
        {   // reference pose at the current time (for tracking-error measurements)
            Eigen::Vector3d pose, ub;
            reference(t0, pose, ub);
            geometry_msgs::PoseStamped rp;
            rp.header.frame_id = "world";
            rp.header.stamp = ros::Time(now);
            rp.pose.position.x = pose(0);
            rp.pose.position.y = pose(1);
            rp.pose.orientation = tf::createQuaternionMsgFromYaw(pose(2));
            ref_pose_pub_.publish(rp);
        }

        // predicted path for rviz (nonlinear rollout of the optimal inputs)
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        Eigen::Vector3d x = x_meas_;
        for (int k = 0; k <= N; k++)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = x(0);
            ps.pose.position.y = x(1);
            ps.pose.position.z = 0.2;
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(x(2));
            path.poses.push_back(ps);
            if (k == N) break;
            const double c = std::cos(x(2)), s = std::sin(x(2));
            const Eigen::Vector3d u = U_prev_.col(k);
            x += dt_ * Eigen::Vector3d(c * u(0) - s * u(1), s * u(0) + c * u(1), u(2));
        }
        path_pub_.publish(path);

        std_msgs::Float64MultiArray st;
        st.data = {std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count(), solve_ms,
                   iters, ok ? 1.0 : 0.0, period_ms};
        stats_pub_.publish(st);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holo_nmpc");
    ros::NodeHandle nh, pnh("~");
    HoloNmpc ctrl(nh, pnh);
    ros::spin();
    return 0;
}
