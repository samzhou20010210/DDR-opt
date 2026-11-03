// nav_bridge: adaptation layer between DDR-opt (planner + MPC + laser simulator) and hull_sim (pool_nav.launch).
//
//   mpc_cmd (carstatemsgs/CarState v, omega)  ->  cmd_vel (geometry_msgs/Twist)        controller -> vessel
//   odom    (nav_msgs/Odometry, hull_sim truth) -> sensor_pose (carstatemsgs/CarState)  vessel pose -> laser_simulator
//
// The planner and the MPC subscribe to hull_sim's odometry directly (both accept nav_msgs/Odometry).
// No latency here (hull_sim applies control.latency) and no TF (hull_sim owns world -> base_link,
// laser_simulator owns world -> laser).

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "carstatemsgs/CarState.h"

class NavBridge {
 public:
  explicit NavBridge(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pose_pub_ = nh.advertise<carstatemsgs::CarState>("sensor_pose", 10);
    cmd_sub_ = nh.subscribe("mpc_cmd", 10, &NavBridge::cmdCb, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("odom", 10, &NavBridge::odomCb, this, ros::TransportHints().tcpNoDelay());
  }

 private:
  void cmdCb(const carstatemsgs::CarState::ConstPtr& m) {
    geometry_msgs::Twist t;
    t.linear.x = m->v;
    t.angular.z = m->omega;
    cmd_pub_.publish(t);
  }
  void odomCb(const nav_msgs::Odometry::ConstPtr& m) {
    carstatemsgs::CarState c;
    c.Header = m->header;   // frame "world": laser_simulator uses it as the parent of its world -> laser TF
    c.x = m->pose.pose.position.x;
    c.y = m->pose.pose.position.y;
    c.yaw = tf::getYaw(m->pose.pose.orientation);
    c.v = m->twist.twist.linear.x;
    c.omega = m->twist.twist.angular.z;
    pose_pub_.publish(c);
  }

  ros::Publisher cmd_pub_, pose_pub_;
  ros::Subscriber cmd_sub_, odom_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nav_bridge");
  ros::NodeHandle nh;
  NavBridge bridge(nh);
  ros::spin();
  return 0;
}
