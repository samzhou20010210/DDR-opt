// hull_sim: simplified 2D hull simulator for the pool rescue vessel.
//
// Input : /pool/cmd_vel (geometry_msgs/Twist) target forward speed linear.x [m/s], yaw rate angular.z [rad/s]
// Model : command latency -> clamp to mode limits (reverse only if enabled) -> rate-limited kinematics:
//           |v| increasing  : max_acceleration      |v| decreasing : max_deceleration
//           omega           : max_yaw_accel (symmetric)
//           v crossing zero : decelerate to 0 first, then accelerate in the new direction
//         unicycle integration (trapezoidal v, omega; midpoint heading). No sway, drag or disturbances.
//         No command for control.cmd_timeout -> target (0, 0).
// Walls : the rectangular hull outline must stay inside [0, L] x [0, W]. A step that would put a hull corner
//         outside is rejected: pose kept, v = omega = 0, /pool/robot/wall_contact = true. Contact is latched
//         while the same target keeps arriving; a different target may move the hull again (clears contact).
//         At startup the standby pose must keep the whole hull inside the pool, otherwise the node exits.
// Output: /pool/robot/odom (nav_msgs/Odometry, world -> base_link, twist in body frame), TF world -> base_link,
//         /pool/hull_sim/target (TwistStamped: target applied after latency + clamping),
//         /pool/robot/wall_contact (Bool), /pool/robot/trajectory (Path), /pool/robot/hull_markers (MarkerArray).
// Service: ~reset (std_srvs/Empty) puts the vessel back to the standby pose at rest.
// Parameters: central /pool/<group>/<name>/value tree (config/pool_rescue.yaml) + /pool/mode.

#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace {
template <typename T>
T getOrDie(const std::string& key) {
  T value;
  if (!ros::param::get(key, value))
    throw std::runtime_error("missing parameter " + key + " (load config/pool_rescue.yaml under /pool)");
  return value;
}
template <typename T>
T poolParam(const std::string& path) { return getOrDie<T>("/pool/" + path + "/value"); }

double approach(double from, double to, double max_step) {
  const double d = to - from;
  return std::fabs(d) <= max_step ? to : from + std::copysign(max_step, d);
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Pose2 { double x, y, yaw; };
}  // namespace

class HullSim {
 public:
  HullSim(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    const std::string mode = getOrDie<std::string>("/pool/mode");
    const std::string m = "modes/" + mode + "/";
    world_ = poolParam<std::string>("frames/world");
    base_ = poolParam<std::string>("frames/base_link");
    L_ = poolParam<double>("pool/length");
    W_ = poolParam<double>("pool/width");
    hl_ = poolParam<double>("vessel/length") / 2.0;
    hw_ = poolParam<double>("vessel/width") / 2.0;
    clearance_ = poolParam<double>("safety/wall_clearance");
    v_fwd_ = poolParam<double>(m + "max_forward_speed");
    reverse_ = poolParam<bool>("vessel/reverse_enabled");
    v_rev_ = reverse_ ? poolParam<double>(m + "max_reverse_speed") : 0.0;
    acc_ = poolParam<double>(m + "max_acceleration");
    dec_ = poolParam<double>(m + "max_deceleration");
    w_max_ = poolParam<double>(m + "max_yaw_rate");
    w_acc_ = poolParam<double>(m + "max_yaw_accel");
    latency_ = poolParam<double>("control/latency");
    cmd_timeout_ = poolParam<double>("control/cmd_timeout");
    rate_ = poolParam<double>("simulation/hull_sim_rate");
    traj_period_ = poolParam<double>("simulation/trajectory_period");
    traj_max_ = poolParam<int>("simulation/trajectory_max_points");
    standby_ = {poolParam<double>("standby/x"), poolParam<double>("standby/y"), poolParam<double>("standby/yaw")};

    // Standby validity: whole hull inside the pool, with the configured wall clearance.
    const double margin = minWallDistance(standby_);
    if (margin < clearance_) {
      throw std::runtime_error("standby pose (" + std::to_string(standby_.x) + ", " + std::to_string(standby_.y) +
                               ") leaves only " + std::to_string(margin) + " m between hull and wall, need " +
                               std::to_string(clearance_) + " m");
    }

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/pool/robot/odom", 10);
    target_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/pool/hull_sim/target", 10);
    contact_pub_ = nh.advertise<std_msgs::Bool>("/pool/robot/wall_contact", 1, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("/pool/robot/trajectory", 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/pool/robot/hull_markers", 1, true);
    cmd_sub_ = nh.subscribe("/pool/cmd_vel", 20, &HullSim::cmdCb, this, ros::TransportHints().tcpNoDelay());
    reset_srv_ = pnh.advertiseService("reset", &HullSim::resetCb, this);

    reset();
    publishMarkers();
    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &HullSim::step, this);
    ROS_INFO("[hull_sim] mode=%s  v=[%.2f, %.2f] m/s  acc/dec=%.2f/%.2f m/s^2  w=%.2f rad/s  w_acc=%.2f rad/s^2  "
             "latency=%.3f s  reverse=%s  standby=(%.2f, %.2f, %.2f)  hull-wall margin=%.2f m",
             mode.c_str(), -v_rev_, v_fwd_, acc_, dec_, w_max_, w_acc_, latency_, reverse_ ? "on" : "off",
             standby_.x, standby_.y, standby_.yaw, margin);
  }

 private:
  // ---------------------------------------------------------------- geometry
  std::array<std::array<double, 2>, 4> corners(const Pose2& p) const {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    std::array<std::array<double, 2>, 4> out;
    const double bx[4] = {hl_, hl_, -hl_, -hl_}, by[4] = {hw_, -hw_, -hw_, hw_};
    for (int i = 0; i < 4; ++i) out[i] = {p.x + c * bx[i] - s * by[i], p.y + s * bx[i] + c * by[i]};
    return out;
  }
  double minWallDistance(const Pose2& p) const {   // < 0 means a corner is outside the pool
    double d = 1e9;
    for (const auto& q : corners(p)) d = std::min({d, q[0], L_ - q[0], q[1], W_ - q[1]});
    return d;
  }

  // ---------------------------------------------------------------- input
  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    queue_.push_back({ros::Time::now() + ros::Duration(latency_), msg->linear.x, msg->angular.z});
  }
  bool resetCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    reset();
    ROS_INFO("[hull_sim] reset to standby");
    return true;
  }
  void reset() {
    pose_ = standby_;
    v_ = w_ = tv_ = tw_ = 0.0;
    queue_.clear();
    last_cmd_ = ros::Time(0);
    last_step_ = ros::Time::now();
    path_.poses.clear();
    path_.header.frame_id = world_;
    last_traj_ = ros::Time(0);
    setContact(false);
  }

  // ---------------------------------------------------------------- simulation step
  void step(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double dt = std::min(std::max((now - last_step_).toSec(), 0.0), 0.05);
    last_step_ = now;

    while (!queue_.empty() && queue_.front().due <= now) {   // commands whose latency has elapsed
      tv_ = queue_.front().v;
      tw_ = queue_.front().w;
      last_cmd_ = queue_.front().due;
      queue_.pop_front();
    }
    if (cmd_timeout_ > 0.0 && (last_cmd_.isZero() || (now - last_cmd_).toSec() > cmd_timeout_)) tv_ = tw_ = 0.0;

    const double tv = std::min(std::max(tv_, -v_rev_), v_fwd_);
    const double tw = std::min(std::max(tw_, -w_max_), w_max_);

    // speed: pass through zero when reversing direction; rate depends on |v| growing or shrinking
    const double v_goal = (v_ * tv < 0.0) ? 0.0 : tv;
    const double v_rate = (std::fabs(v_goal) > std::fabs(v_)) ? acc_ : dec_;
    const double v_new = approach(v_, v_goal, v_rate * dt);
    const double w_new = approach(w_, tw, w_acc_ * dt);

    // Contact is latched: while the target that drove the hull into the wall is unchanged, stay stopped.
    // Any different target (e.g. turning away, reversing, stopping) may try to move again.
    if (contact_ && tv == contact_tv_ && tw == contact_tw_) {
      v_ = w_ = 0.0;
      publish(now, tv, tw);
      return;
    }

    const double v_avg = 0.5 * (v_ + v_new), w_avg = 0.5 * (w_ + w_new);
    Pose2 next{0, 0, 0};
    const double yaw_mid = pose_.yaw + 0.5 * w_avg * dt;
    next.x = pose_.x + v_avg * dt * std::cos(yaw_mid);
    next.y = pose_.y + v_avg * dt * std::sin(yaw_mid);
    next.yaw = wrap(pose_.yaw + w_avg * dt);

    if (minWallDistance(next) < 0.0) {           // hull would cross a wall: stop at the current pose
      v_ = w_ = 0.0;
      if (!contact_) ROS_WARN("[hull_sim] wall contact at (%.2f, %.2f, %.2f): vessel stopped", pose_.x, pose_.y, pose_.yaw);
      contact_tv_ = tv;
      contact_tw_ = tw;
      if (!contact_) setContact(true);
    } else {
      pose_ = next;
      v_ = v_new;
      w_ = w_new;
      if (contact_) setContact(false);
    }
    publish(now, tv, tw);
  }

  // ---------------------------------------------------------------- output
  void setContact(bool c) {
    contact_ = c;
    std_msgs::Bool b;
    b.data = c;
    contact_pub_.publish(b);
  }

  void publish(const ros::Time& stamp, double tv, double tw) {
    nav_msgs::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = world_;
    o.child_frame_id = base_;
    o.pose.pose.position.x = pose_.x;
    o.pose.pose.position.y = pose_.y;
    o.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.yaw);
    o.twist.twist.linear.x = v_;     // body frame
    o.twist.twist.angular.z = w_;
    odom_pub_.publish(o);

    geometry_msgs::TransformStamped t;
    t.header = o.header;
    t.child_frame_id = base_;
    t.transform.translation.x = pose_.x;
    t.transform.translation.y = pose_.y;
    t.transform.rotation = o.pose.pose.orientation;
    tf_br_.sendTransform(t);

    geometry_msgs::TwistStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = base_;
    ts.twist.linear.x = tv;
    ts.twist.angular.z = tw;
    target_pub_.publish(ts);

    if ((stamp - last_traj_).toSec() >= traj_period_) {
      last_traj_ = stamp;
      geometry_msgs::PoseStamped p;
      p.header = o.header;
      p.pose = o.pose.pose;
      path_.poses.push_back(p);
      if (static_cast<int>(path_.poses.size()) > traj_max_) path_.poses.erase(path_.poses.begin());
      path_.header.stamp = stamp;
      path_pub_.publish(path_);
    }
  }

  void publishMarkers() {   // static shapes in base_link: RViz moves them with TF
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker hull;
    hull.header.frame_id = base_;
    hull.ns = "hull";
    hull.id = 0;
    hull.type = visualization_msgs::Marker::LINE_STRIP;
    hull.action = visualization_msgs::Marker::ADD;
    hull.pose.orientation.w = 1.0;
    hull.scale.x = 0.04;
    hull.color.r = 1.0; hull.color.g = 0.45; hull.color.b = 0.0; hull.color.a = 1.0;
    const double bx[5] = {hl_, hl_, -hl_, -hl_, hl_}, by[5] = {hw_, -hw_, -hw_, hw_, hw_};
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::Point p;
      p.x = bx[i]; p.y = by[i]; p.z = 0.05;
      hull.points.push_back(p);
    }
    arr.markers.push_back(hull);

    visualization_msgs::Marker arrow = hull;   // heading
    arrow.ns = "heading";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.points.clear();
    geometry_msgs::Point a, b;
    a.z = b.z = 0.06;
    b.x = hl_ + 0.3;
    arrow.points = {a, b};
    arrow.scale.x = 0.05; arrow.scale.y = 0.12; arrow.scale.z = 0.12;
    arrow.color.r = 1.0; arrow.color.g = 0.1; arrow.color.b = 0.1;
    arr.markers.push_back(arrow);
    marker_pub_.publish(arr);
  }

  struct Cmd { ros::Time due; double v, w; };
  std::string world_, base_;
  double L_, W_, hl_, hw_, clearance_, v_fwd_, v_rev_, acc_, dec_, w_max_, w_acc_, latency_, cmd_timeout_, rate_;
  double traj_period_;
  int traj_max_;
  bool reverse_;
  Pose2 standby_, pose_;
  double v_ = 0, w_ = 0, tv_ = 0, tw_ = 0;
  double contact_tv_ = 0, contact_tw_ = 0;   // target that caused the latched wall contact
  bool contact_ = false;
  std::deque<Cmd> queue_;
  ros::Time last_cmd_, last_step_, last_traj_;
  nav_msgs::Path path_;
  ros::Publisher odom_pub_, target_pub_, contact_pub_, path_pub_, marker_pub_;
  ros::Subscriber cmd_sub_;
  ros::ServiceServer reset_srv_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hull_sim");
  ros::NodeHandle nh, pnh("~");
  try {
    HullSim sim(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[hull_sim] %s", e.what());
    return 1;
  }
  return 0;
}
