#!/usr/bin/env python3
"""Bridge between DDR-opt's planar holonomic ROV stack (robot:=rov_holo) and ROV_V4 in the uwv-simulator Gazebo pool.

In  /rov_v4/sim_ground_truth (nav_msgs/Odometry, Gazebo world, p3d: pose and world-frame twist of rov_v4_base_link)
    /thruster_allocation/thrust (Float64MultiArray, 8 thrusts in rov_thrusters.yaml order, N)
    ~depth_target (std_msgs/Float64, m below the surface)
Out ~odom  (nav_msgs/Odometry, frame "world" = pool corner, child base_link): planar pose (x, y, yaw) and the
           heading-frame body twist (vx surge, vy sway, omega yaw rate), the convention of the 2D simulator's
           /simulation/PoseOdomPub; position.z = Gazebo z (not used by the planner)
    ~pose  (PoseStamped, same pose, for tests)      ~car_state (carstatemsgs/CarState x, y, yaw for laser_simulator)
    ~zkm   (WrenchStamped force.z, torque.x, torque.y): depth / attitude hold targets for thruster_allocation
    /rov_v4/thruster_command (sensor_msgs/JointState, Gazebo names, effort N), matched BY NAME
    tf world -> base_link, static tf world -> gazebo_world

Safety: Gazebo applies the last thruster command forever, so thrust goes to 0 when the allocation stops publishing
(thrust_timeout) and on shutdown. An emergency stop in DDR-opt only stops the planar motion (the NMPC stops
commanding, the allocation then commands zero planar wrench) and keeps holding depth.
"""
import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from carstatemsgs.msg import CarState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Bridge:
    def __init__(self):
        g = lambda k, d=None: rospy.get_param("~" + k, d)
        self.off = g("pool_offset", [12.5, 25.0])
        self.tmap = g("thruster_map")
        self.ddr_names = rospy.get_param("/thruster_allocation/thrusters/names", None) or g("thrusters/names")
        missing = [n for n in self.ddr_names if n not in self.tmap]
        if missing:
            raise RuntimeError("no Gazebo name for DDR-opt thrusters %s" % missing)
        self.gz_names = [self.tmap[n] for n in self.ddr_names]
        self.timeout = g("thrust_timeout", 0.2)
        d = g("depth")
        a = g("attitude")
        self.depth_target, self.kp, self.ki, self.kd = d["target"], d["kp"], d["ki"], d["kd"]
        self.i_lim, self.zmax, self.zff = d["i_limit"], d["max_force"], d["buoyancy_ff"]
        self.akp, self.akd, self.amax = a["kp"], a["kd"], a["max_moment"]
        self.integ, self.t_prev = 0.0, None
        self.last_thrust = None
        self.zero_sent = False
        self.closing = False

        self.odom_pub = rospy.Publisher("~odom", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=10)
        self.car_pub = rospy.Publisher("~car_state", CarState, queue_size=10)
        self.zkm_pub = rospy.Publisher("~zkm", WrenchStamped, queue_size=10)
        self.depth_pub = rospy.Publisher("~depth", Float64, queue_size=10)
        self.cmd_pub = rospy.Publisher("/rov_v4/thruster_command", JointState, queue_size=10)
        self.tf = tf2_ros.TransformBroadcaster()
        st = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id, t.child_frame_id = "world", "gazebo_world"
        t.transform.translation.x, t.transform.translation.y = self.off
        t.transform.rotation.w = 1.0
        st.sendTransform(t)

        rospy.Subscriber("/rov_v4/sim_ground_truth", Odometry, self.cb_gt, queue_size=10)
        rospy.Subscriber("/thruster_allocation/thrust", Float64MultiArray, self.cb_thrust, queue_size=10)
        rospy.Subscriber("~depth_target", Float64, lambda m: setattr(self, "depth_target", m.data))
        rospy.Timer(rospy.Duration(0.02), self.watchdog)
        # Zero the thrusters BEFORE rospy closes its connections. rospy runs its client shutdown hooks (which drop the
        # topic connections) before the pre-shutdown and on_shutdown hooks, so zeros sent from those never reach
        # Gazebo, which then keeps the last thrust forever (verified with run_headless_gazebo.sh). Put the hook first
        # in that (private) list; fall back to the public pre-shutdown hook if the internals change.
        try:
            rospy.core._client_shutdown_hooks.insert(0, self.shutdown)
        except AttributeError:
            rospy.core.add_preshutdown_hook(self.shutdown)
        rospy.loginfo("[gazebo_bridge] pool offset %s, depth target %.2f m, thrusters %s",
                      self.off, self.depth_target, list(zip(self.ddr_names, self.gz_names)))

    # ------------------------------------------------------------------ state in
    def cb_gt(self, m):
        q = m.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        vw = m.twist.twist.linear
        ww = m.twist.twist.angular
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = c * vw.x + s * vw.y, -s * vw.x + c * vw.y          # heading frame (planar model)
        wb = [R[0][k] * ww.x + R[1][k] * ww.y + R[2][k] * ww.z for k in range(3)]   # body angular rates
        x, y, z = m.pose.pose.position.x + self.off[0], m.pose.pose.position.y + self.off[1], m.pose.pose.position.z
        stamp = m.header.stamp

        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "world", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        qy = quaternion_from_euler(0, 0, yaw)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = qy
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, ww.z
        self.odom_pub.publish(o)
        p = PoseStamped()
        p.header = o.header
        p.pose = o.pose.pose
        self.pose_pub.publish(p)
        cs = CarState()
        cs.Header.stamp, cs.Header.frame_id = stamp, "world"
        cs.x, cs.y, cs.yaw, cs.v, cs.omega = x, y, yaw, vx, ww.z
        self.car_pub.publish(cs)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "world", "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation = q
        self.tf.sendTransform(t)
        self.depth_pub.publish(Float64(-z))

        # depth (z up, depth = -z) and attitude hold -> (Z, K, M) in the body frame
        now = stamp.to_sec()
        dt = clamp(now - self.t_prev, 0.0, 0.1) if self.t_prev is not None else 0.0
        self.t_prev = now
        ez = -self.depth_target - z
        self.integ = clamp(self.integ + self.ki * ez * dt, -self.i_lim, self.i_lim)
        Zw = clamp(self.kp * ez + self.integ - self.kd * vw.z + self.zff, -self.zmax, self.zmax)
        Zb = Zw * R[2][2]                                            # world-z force expressed along body z
        K = clamp(-self.akp * roll - self.akd * wb[0], -self.amax, self.amax)
        M = clamp(-self.akp * pitch - self.akd * wb[1], -self.amax, self.amax)
        w = WrenchStamped()
        w.header.stamp, w.header.frame_id = stamp, "base_link"
        w.wrench.force.z, w.wrench.torque.x, w.wrench.torque.y = Zb, K, M
        self.zkm_pub.publish(w)

    # ------------------------------------------------------------------ thrust out
    def cb_thrust(self, m):
        if self.closing:
            return
        if len(m.data) != len(self.gz_names):
            rospy.logwarn_throttle(1.0, "[gazebo_bridge] %d thrusts for %d thrusters, ignored", len(m.data), len(self.gz_names))
            return
        self.last_thrust = rospy.get_time()
        self.zero_sent = False
        self.send(list(m.data))

    def send(self, efforts):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(self.gz_names)
        js.effort = [float(e) for e in efforts]
        self.cmd_pub.publish(js)

    def watchdog(self, _):
        if self.last_thrust is not None and rospy.get_time() - self.last_thrust > self.timeout and not self.zero_sent:
            rospy.logwarn("[gazebo_bridge] no thrust command for %.2f s: all thrusters 0", self.timeout)
            self.send([0.0] * len(self.gz_names))
            self.zero_sent = True

    def shutdown(self, reason=""):
        if self.closing:
            return
        self.closing = True     # stop forwarding allocation output, then zero every thruster
        for _ in range(10):
            self.send([0.0] * len(self.gz_names))
            time.sleep(0.02)
        rospy.loginfo("[gazebo_bridge] shutdown: all thrusters 0")


if __name__ == "__main__":
    rospy.init_node("gazebo_bridge")
    Bridge()
    rospy.spin()
