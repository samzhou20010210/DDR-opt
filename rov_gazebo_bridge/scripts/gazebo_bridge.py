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

Stops (Gazebo applies the last thruster command forever, so "off" means actively sending zeros):
  SAFETY stop -> all 8 thrusters 0, LATCHED; allocation output is ignored; /safety_stop (latched Bool) tells the
    planner (drops the goal) and holo_nmpc (stops). Triggers:
      - manual: /rov_safety/estop (Bool true)
      - collision risk: /safety_stop true from the planner (robot centre inside an obstacle)
      - comm loss: no /thruster_allocation/thrust for thrust_timeout, or no ground truth for odom_timeout
    Re-enable ONLY by manual reset: rosservice call /gazebo_bridge/reset_safety_stop
  PLANNING stop (/planner/emergency_stop: replan failure, goal dropped) is not handled here: the NMPC stops
    commanding, the allocation then commands zero planar wrench and keeps holding depth / attitude; the next valid
    goal resumes.
  Shutdown of this node -> all 8 thrusters 0.
"""
import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from carstatemsgs.msg import CarState
from visualization_msgs.msg import Marker
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
        self.odom_timeout = g("odom_timeout", 0.5)
        self.safety = None          # reason of a latched safety stop, None = running
        self.last_gt = None
        self.last_zero = 0.0
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
        self.safety_pub = rospy.Publisher("/safety_stop", Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~robot_marker", Marker, queue_size=1)   # ROV_V4 mesh for rviz
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
        rospy.Subscriber("/rov_safety/estop", Bool, lambda m: m.data and self.trip("manual e-stop"))
        rospy.Subscriber("/safety_stop", Bool, lambda m: m.data and self.trip("collision risk (planner)"))
        rospy.Service("~reset_safety_stop", Trigger, self.reset)
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
        self.last_gt = rospy.get_time()
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
        mk = Marker(type=Marker.MESH_RESOURCE, action=Marker.ADD, ns="rov", id=0,
                    mesh_resource="package://uwv_description/meshes/rov_v4/ROV_V4.STL")
        mk.header.stamp, mk.header.frame_id = stamp, "world"
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
        mk.pose.orientation = q                                     # full 3D attitude
        mk.scale.x = mk.scale.y = mk.scale.z = 1.0
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.7, 0.2, 1.0
        self.marker_pub.publish(mk)

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
        if self.closing or self.safety:
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

    # ------------------------------------------------------------------ safety stop
    def trip(self, reason):
        if self.closing:
            return
        if self.safety is None:
            self.safety = reason
            rospy.logerr("[gazebo_bridge] SAFETY STOP (%s): all 8 thrusters 0, latched. Reset: "
                         "rosservice call /gazebo_bridge/reset_safety_stop", reason)
            self.safety_pub.publish(Bool(True))
        self.send([0.0] * len(self.gz_names))
        self.zero_sent = True

    def reset(self, _req):
        was = self.safety
        self.safety = None
        self.last_thrust = None      # comm-loss check re-arms with the next thrust command
        self.zero_sent = False
        self.integ = 0.0
        self.safety_pub.publish(Bool(False))
        rospy.logwarn("[gazebo_bridge] safety stop reset (was: %s)", was)
        return TriggerResponse(success=True, message="reset (was: %s)" % was)

    def watchdog(self, _):
        now = rospy.get_time()
        if self.safety:
            if now - self.last_zero > 0.5:      # keep re-sending zeros while latched
                self.send([0.0] * len(self.gz_names))
                self.last_zero = now
            return
        if self.last_thrust is not None and now - self.last_thrust > self.timeout:
            self.trip("comm loss: no thrust command for %.2f s" % self.timeout)
        elif self.last_gt is not None and now - self.last_gt > self.odom_timeout:
            self.trip("comm loss: no ground truth for %.2f s" % self.odom_timeout)

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
