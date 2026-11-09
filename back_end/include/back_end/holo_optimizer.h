#ifndef _HOLO_OPTIMIZER_H_
#define _HOLO_OPTIMIZER_H_

// Planar holonomic trajectory optimizer for the ROV (robot:=rov_holo).
//
// Flat output sigma(t) = (x, y, theta) in the world frame, one MINCO minimum-jerk polynomial per dimension
// (minco::MINCO_S3NU_ND<3>). Kinematics (body frame b, x forward / y port):
//     xdot = vx cos(theta) - vy sin(theta),  ydot = vx sin(theta) + vy cos(theta),  thetadot = omega
// so (vx, vy) = R(theta)^T (xdot, ydot) and omega = thetadot are read straight off the polynomial; there is no
// arc-length integration, no ICR slip term and no wheel-speed coupling (those stay in MSPlanner for the car / rov).
//
// Initial guess: the JPS path geometry resampled by arc length, heading along the path, surge-limit timing.
// Decision variables: inner waypoints (3 x (N-1)) and piece durations (N, via the same virtual-time map as
// MSPlanner). Start and goal pose/velocity/acceleration are hard MINCO boundary conditions, so the PHR-ALM
// terminal-position equality that MSPlanner needs (its x-y come from integration) is not required here.
// Inequalities use the same smoothed-L1 penalty scheme as MSPlanner, sampled sparseResolution times per piece:
//   independent body limits  min_vx <= vx <= max_vx, |vy| <= max_vy, |omega| <= max_omega
//   body accelerations       |ax_b| <= max_ax, |ay_b| <= max_ay, |omegadot| <= max_domega   (a_b = R^T (xddot, yddot))
//   thruster reachability    tau = M a_b + D v_b  must lie inside the zonotope {B f : f_min <= f <= f_max} of the
//                            horizontal thrusters (shrunk by alloc_reserve); B from the thruster layout
//   ESDF safety              sdf(p + R c_k) >= safeDis for every body check point c_k
// ALL dynamic numbers (limits, M, D, f_min/f_max) are PLACEHOLDERS until mass, inertia, drag and T500 thrust curves
// exist; a trajectory that satisfies them is NOT shown to be dynamically feasible for the real vehicle.

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <memory>
#include <string>
#include <vector>

#include "plan_env/sdf_map.h"
#include "front_end/traj_representation.h"
#include "gcopter/trajectory.hpp"
#include "gcopter/minco_holo.hpp"
#include "gcopter/lbfgs.hpp"

struct HoloLimits
{
    double max_vx, min_vx, max_vy, max_omega;
    double max_ax, max_ay, max_domega;
};

struct HoloWeights
{
    double time, vel, omega, acc, domega, collision, alloc, mean_time;
};

class HoloPlanner
{
public:
    typedef Eigen::Matrix<double, 3, 3> PVA;   // rows x, y, theta; cols position, velocity, acceleration

    HoloPlanner(ros::NodeHandle &nh, std::shared_ptr<SDFmap> map);

    // Plan from head (pose/vel/acc in the world frame) through the JPS samples in flat_traj to its final pose.
    bool plan(const FlatTrajData &flat_traj, const PVA &head);

    // Results
    Trajectory<5, 3> final_traj_;
    PVA head_, tail_;
    Eigen::Matrix3Xd final_inner_;
    Eigen::VectorXd final_times_;

    // State of the current trajectory at time t from its start (clamped): world pose / velocity / acceleration
    PVA getPVA(double t) const;
    // Poses between t0 and t1 at trajPredictResolution, the last one is the pose at t1
    std::vector<Eigen::Vector3d> getPath(double t0, double t1) const;
    // Body-frame (vx, vy, omega) of the current trajectory at time t
    Eigen::Vector3d getBodyVel(double t) const;
    // Smallest obstacle distance of the current trajectory from time t to its end (current map)
    double remainingClearance(double t) const;
    double finalMinSafeDis() const { return final_min_safe_dis_; }

    void pathPub(const ros::Publisher &pub) const;
    void pointPub(const ros::Publisher &pub, const Eigen::Vector3d &color) const;

    double safeDis() const { return safe_dis_; }

    // Finite-difference check of the analytic gradient on the current problem (debug, param holo/gradient_check)
    void gradientCheck(const Eigen::VectorXd &x);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<SDFmap> map_;

    HoloLimits lim_;
    HoloWeights wt_, wt_backup_;
    Eigen::Vector3d energy_w_;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    double smooth_eps_;
    double mean_time_low_, mean_time_upp_;
    int K_;                                 // samples per piece (sparseResolution)
    double safe_dis_, safe_dis_cfg_, final_min_safe_dis_;
    int final_check_num_, safe_replan_max_;
    double predict_res_;
    int mintraj_num_;
    double piece_len_;     // initial-guess piece length along the path (m)
    double limit_tol_;     // accepted relative excess over a limit (verification) before re-optimizing
    double limit_margin_;  // penalties target (1 - margin) x each limit (and margin inside the thrust facets)
    int max_attempts_;
    std::vector<Eigen::Vector2d> check_points_;
    bool gradient_check_;

    // allocation reachability (placeholder dynamics)
    bool alloc_enable_;
    double mass_, inertia_z_;
    Eigen::Vector3d drag_;                  // linear drag (X/u, Y/v, N/r)
    std::vector<Eigen::Vector3d> facet_n_;  // facets n^T tau <= h of the (shrunk) planar thrust zonotope
    std::vector<double> facet_h_;

    // problem data
    int N_;
    PVA head_cur_, tail_cur_;
    Eigen::Matrix3Xd inner_;
    Eigen::VectorXd times_;
    minco::MINCO_S3NU_ND<3> minco_;
    minco::MINCO_S3NU_ND<3>::CoeffMat gdC_;
    Eigen::VectorXd gdT_;
    Eigen::Matrix3Xd gradByPoints_;
    Eigen::VectorXd gradByTimes_;
    int iter_num_;
    std::vector<Eigen::Vector2d> collision_points_;

    // per-term costs for logging
    double c_energy_, c_time_, c_vel_, c_acc_, c_alloc_, c_coll_, c_meanT_;

    void buildAllocationFacets();
    bool initialGuess(const FlatTrajData &flat, const PVA &head);
    bool optimize();
    bool stretchTime(double excess);
    bool checkFinalCollision(double &min_dis, double min_ok) const;
    double limitExcess(const Trajectory<5, 3> &traj, bool log) const;

    double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g);
    void attachPenalty(double &cost);
    static double costCallback(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);

    inline void smoothL1(const double &x, double &f, double &df) const;
    static void realT2Virtual(const Eigen::VectorXd &RT, Eigen::Ref<Eigen::VectorXd> VT);
    static void virtualT2Real(const Eigen::Ref<const Eigen::VectorXd> &VT, Eigen::VectorXd &RT);
    static void backwardGradT(const Eigen::Ref<const Eigen::VectorXd> &tau, const Eigen::VectorXd &gradT,
                              Eigen::Ref<Eigen::VectorXd> gradTau);

    template <typename T>
    void param(const std::string &name, T &value, const T &def)
    {
        if (!nh_.getParam(ros::this_node::getName() + "/" + name, value))
        {
            value = def;
            ROS_WARN("[holo] param %s not set, using default", name.c_str());
        }
    }
};

#endif
