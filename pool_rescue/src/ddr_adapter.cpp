// ddr_adapter: the only translation layer between legacy DDR-opt messages and the
// pool_rescue interface conventions (see pool_rescue/README.md). Legacy nodes are unchanged.
//
//  controller side : mpc_cmd   (carstatemsgs/CarState v, omega) -> cmd_vel (geometry_msgs/Twist)
//  vessel side     : cmd_vel   (Twist) --[latency]--> sim_cmd (CarState, simulator input)
//  state           : ekf_odom  (nav_msgs/Odometry, pose only) + truth velocity -> odom (Odometry)
//                    truth     (CarState, simulator ground truth)              -> odom_truth (Odometry)
//  TF              : world -> base_link from `odom` (this node is its single publisher)
//
// Written in C++ (not rospy) because the simulator subscribes to its command topic with
// TransportHints().unreliable() (UDPROS), which rospy publishers do not support.

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "carstatemsgs/CarState.h"

class DdrAdapter {
 public:
  explicit DdrAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("latency", latency_, 0.0);
    pnh.param<std::string>("world_frame", world_frame_, "world");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    if (latency_ < 0.0) { ROS_WARN("[ddr_adapter] negative latency %.3f -> 0", latency_); latency_ = 0.0; }

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    sim_cmd_pub_ = nh.advertise<carstatemsgs::CarState>("sim_cmd", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    odom_truth_pub_ = nh.advertise<nav_msgs::Odometry>("odom_truth", 10);

    mpc_cmd_sub_ = nh.subscribe("mpc_cmd", 10, &DdrAdapter::mpcCmdCb, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &DdrAdapter::cmdVelCb, this);
    ekf_sub_ = nh.subscribe("ekf_odom", 10, &DdrAdapter::ekfCb, this);
    truth_sub_ = nh.subscribe("truth", 10, &DdrAdapter::truthCb, this);
    delay_timer_ = nh.createTimer(ros::Duration(0.001), &DdrAdapter::releaseCb, this);

    ROS_INFO("[ddr_adapter] latency=%.3f s, frames %s -> %s", latency_, world_frame_.c_str(), base_frame_.c_str());
  }

 private:
  // ---- controller side: CarState (v, omega) -> Twist --------------------------------
  void mpcCmdCb(const carstatemsgs::CarState::ConstPtr& m) {
    geometry_msgs::Twist t;
    t.linear.x = m->v;        // body-frame forward speed [m/s]
    t.angular.z = m->omega;   // yaw rate [rad/s]
    cmd_vel_pub_.publish(t);
  }

  // ---- vessel side: Twist -> (latency) -> simulator CarState ------------------------
  void cmdVelCb(const geometry_msgs::Twist::ConstPtr& m) {
    queue_.push_back({ros::Time::now() + ros::Duration(latency_), *m});
    if (latency_ <= 0.0) releaseCb(ros::TimerEvent());
  }
  void releaseCb(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    while (!queue_.empty() && queue_.front().due <= now) {
      carstatemsgs::CarState c;
      c.Header.frame_id = world_frame_;
      c.Header.stamp = now;
      c.v = queue_.front().twist.linear.x;
      c.omega = queue_.front().twist.angular.z;
      sim_cmd_pub_.publish(c);
      queue_.pop_front();
    }
  }

  // ---- state ------------------------------------------------------------------------
  void truthCb(const carstatemsgs::CarState::ConstPtr& m) {
    truth_v_ = m->v;
    truth_omega_ = m->omega;
    nav_msgs::Odometry o;
    o.header.stamp = m->Header.stamp;
    o.header.frame_id = world_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = m->x;
    o.pose.pose.position.y = m->y;
    o.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m->yaw);
    o.twist.twist.linear.x = m->v;
    o.twist.twist.angular.z = m->omega;
    odom_truth_pub_.publish(o);
  }
  void ekfCb(const nav_msgs::Odometry::ConstPtr& m) {
    nav_msgs::Odometry o = *m;
    o.header.frame_id = world_frame_;
    o.child_frame_id = base_frame_;
    // The ICR-EKF estimates pose only; in simulation the body twist comes from the simulator.
    o.twist.twist.linear.x = truth_v_;
    o.twist.twist.angular.z = truth_omega_;
    odom_pub_.publish(o);

    geometry_msgs::TransformStamped tf;
    tf.header = o.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = o.pose.pose.position.x;
    tf.transform.translation.y = o.pose.pose.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = o.pose.pose.orientation;
    if (tf.header.stamp != last_tf_stamp_) {   // avoid TF_REPEATED_DATA warnings
      tf_br_.sendTransform(tf);
      last_tf_stamp_ = tf.header.stamp;
    }
  }

  struct Pending { ros::Time due; geometry_msgs::Twist twist; };
  std::deque<Pending> queue_;
  double latency_;
  std::string world_frame_, base_frame_;
  double truth_v_ = 0.0, truth_omega_ = 0.0;
  ros::Time last_tf_stamp_;
  ros::Publisher cmd_vel_pub_, sim_cmd_pub_, odom_pub_, odom_truth_pub_;
  ros::Subscriber mpc_cmd_sub_, cmd_vel_sub_, ekf_sub_, truth_sub_;
  ros::Timer delay_timer_;
  tf2_ros::TransformBroadcaster tf_br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ddr_adapter");
  ros::NodeHandle nh, pnh("~");
  DdrAdapter adapter(nh, pnh);
  ros::spin();
  return 0;
}
