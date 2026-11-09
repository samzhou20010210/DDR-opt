#!/usr/bin/env python3
"""Pool scene for DDR-opt in Gazebo (config/pool_scene.yaml), in the pool-corner frame ("world").

- Gazebo: spawns the lane lines (visual only: no collision) and the obstacles (floor-to-surface columns; static ones
  as static models, dynamic ones as kinematic models without gravity), then moves the dynamic obstacles along their
  scripted paths (/gazebo/set_model_state) as a function of sim time.
- Planner: publishes ~cloud, the pool walls plus every obstacle at its CURRENT position, in the format of the 2D
  simulator's /global_map/laser_gridmap (one column of points per occupied 0.1 m cell, z in [-0.5, 0.4]).
  laser_simulator (dynamic_map: true) ray-casts it into the planner's local point cloud. Lane lines are NOT in it.
- RViz / tests: ~markers (MarkerArray): lane lines (ns "lanes") and obstacles (ns "obstacles"; CYLINDER / CUBE with
  the current pose and size), used by multigoal.py --gazebo to measure clearance to moving obstacles.

~scene: "default" (pool_scene.yaml obstacles) or "empty" (walls and lanes only).
"""
import math

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

RES = 0.1                                   # m, grid of the obstacle cloud (= planner gridmap_interval)
ZS = np.arange(-5, 5) * RES                 # z layers of each occupied cell (as global_map_node)


def cloud_msg(cells, stamp):
    """PointCloud2 (x, y, z float32) with one column of ZS points per cell, packed with numpy"""
    pts = np.empty((len(cells) * len(ZS), 3), dtype=np.float32)
    pts[:, :2] = np.repeat(cells, len(ZS), axis=0)
    pts[:, 2] = np.tile(ZS, len(cells))
    m = PointCloud2(header=Header(frame_id="world", stamp=stamp), height=1, width=len(pts), is_dense=True,
                    is_bigendian=False, point_step=12, row_step=12 * len(pts))
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    m.data = pts.tobytes()
    return m


def obstacle_cells(o, x, y):
    """grid cell centres covered by obstacle o centred at (x, y)"""
    if o["shape"] == "cylinder":
        r = o["radius"]
        hx = hy = r
    else:
        hx, hy = o["size_x"] / 2.0, o["size_y"] / 2.0
    gx = np.arange(math.floor((x - hx) / RES), math.ceil((x + hx) / RES)) * RES + RES / 2
    gy = np.arange(math.floor((y - hy) / RES), math.ceil((y + hy) / RES)) * RES + RES / 2
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    if o["shape"] == "cylinder":
        m = (X - x) ** 2 + (Y - y) ** 2 <= (r + RES / 2) ** 2
    else:
        m = (np.abs(X - x) <= hx + RES / 2) & (np.abs(Y - y) <= hy + RES / 2)
    return np.stack([X[m], Y[m]], axis=1)


def position(o, t):
    """current (x, y) of obstacle o at sim time t"""
    if "position" in o:
        return tuple(o["position"])
    if o["motion"] == "line":
        a, b = np.array(o["a"], float), np.array(o["b"], float)
        L = np.linalg.norm(b - a)
        s = (o["speed"] * t) % (2 * L)
        f = s / L if s <= L else 2 - s / L
        p = a + f * (b - a)
        return float(p[0]), float(p[1])
    if o["motion"] == "circle":
        c, R = o["centre"], o["radius_path"]
        th = o.get("phase", 0.0) + o["speed"] / R * t
        return c[0] + R * math.cos(th), c[1] + R * math.sin(th)
    raise ValueError("unknown motion %s" % o["motion"])


def sdf_column(name, o, depth, static, colour):
    if o["shape"] == "cylinder":
        geom = "<cylinder><radius>%f</radius><length>%f</length></cylinder>" % (o["radius"], depth)
    else:
        geom = "<box><size>%f %f %f</size></box>" % (o["size_x"], o["size_y"], depth)
    link_extra = "" if static else "<gravity>0</gravity><kinematic>1</kinematic>"
    return """<?xml version="1.0"?><sdf version="1.6"><model name="%s"><static>%d</static>
  <link name="link">%s<pose>0 0 %f 0 0 0</pose>
    <collision name="c"><geometry>%s</geometry></collision>
    <visual name="v"><geometry>%s</geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>
  </link></model></sdf>""" % (name, 1 if static else 0, link_extra, -depth / 2.0, geom, geom, colour, colour)


def sdf_lanes(xs, length, radius, z):
    vis = ""
    for i, x in enumerate(xs):
        for k in range(10):                       # 5 m segments, alternating colours (racing-lane style)
            col = "0.1 0.2 0.9 1" if k % 2 == 0 else "0.95 0.95 0.95 1"
            vis += """<visual name="lane%d_%d"><pose>%f %f %f 1.5708 0 0</pose>
      <geometry><cylinder><radius>%f</radius><length>5.0</length></cylinder></geometry>
      <material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>""" % (
                i, k, x, 2.5 + 5.0 * k, z, radius, col, col)
    return """<?xml version="1.0"?><sdf version="1.6"><model name="pool_lane_lines"><static>1</static>
  <link name="link">%s</link></model></sdf>""" % vis


class Scene:
    def __init__(self):
        g = lambda k, d=None: rospy.get_param("~" + k, d)
        self.off = g("pool_offset", [12.5, 25.0])
        pool, lanes = g("pool"), g("lanes")
        self.W, self.L, self.D = pool["width"], pool["length"], pool["depth"]
        n, w = lanes["count"], lanes["width"]
        self.lane_x = [w * k for k in range(1, n)]                  # n lanes -> n - 1 lane lines
        self.lane_r, self.lane_z = lanes["rope_radius"], lanes["rope_z"]
        obs = g("obstacles", {}) if g("scene", "default") != "empty" else {}
        self.static = obs.get("static", []) or []
        self.dynamic = obs.get("dynamic", []) or []
        self.rate = g("update_rate", 20.0)
        self.spawned = False      # no model-state updates before the models exist
        # walls (outside the water, 0.3 m thick), fixed
        th = 0.3
        xs = np.arange(-th + RES / 2, self.W + th, RES)
        ys = np.arange(-th + RES / 2, self.L + th, RES)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        wall = (X < 0) | (X > self.W) | (Y < 0) | (Y > self.L)
        cells = [np.stack([X[wall], Y[wall]], axis=1)]
        cells += [obstacle_cells(o, *o["position"]) for o in self.static]
        self.fixed_cells = np.concatenate(cells)
        self.cloud_pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=20)
        rospy.loginfo("[pool_scene] pool %.0f x %.0f x %.1f m, %d lane lines (visual only), %d static + %d dynamic "
                      "obstacles", self.W, self.L, self.D, len(self.lane_x), len(self.static), len(self.dynamic))

    def gz(self, x, y):
        return x - self.off[0], y - self.off[1]

    def spawn(self):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
        except rospy.ROSException:
            rospy.logerr("[pool_scene] /gazebo/spawn_sdf_model not available: nothing spawned in Gazebo")
            return
        sp = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        p = Pose()
        p.position.x, p.position.y = self.gz(0.0, 0.0)
        p.orientation.w = 1.0
        sp("pool_lane_lines", sdf_lanes(self.lane_x, self.L, self.lane_r, self.lane_z), "", p, "world")
        for o in self.static:
            p = Pose()
            p.position.x, p.position.y = self.gz(*o["position"])
            p.orientation.w = 1.0
            sp(o["name"], sdf_column(o["name"], o, self.D, True, "0.8 0.4 0.1 1"), "", p, "world")
        for o in self.dynamic:
            p = Pose()
            p.position.x, p.position.y = self.gz(*position(o, rospy.get_time()))
            p.orientation.w = 1.0
            sp(o["name"], sdf_column(o["name"], o, self.D, False, "0.9 0.1 0.1 1"), "", p, "world")
        self.spawned = True
        rospy.loginfo("[pool_scene] spawned lane lines and %d obstacles in Gazebo", len(self.static) + len(self.dynamic))

    def markers(self, now, dyn_pos):
        ma = MarkerArray()
        h = Header(frame_id="world", stamp=now)
        for i, x in enumerate(self.lane_x):
            m = Marker(header=h, ns="lanes", id=i, type=Marker.CYLINDER, action=Marker.ADD)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, self.L / 2, 0.0
            m.pose.orientation.x, m.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
            m.scale.x = m.scale.y = 2 * self.lane_r
            m.scale.z = self.L
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.3, 1.0, 0.8
            ma.markers.append(m)
        allo = [(o, tuple(o["position"]), False) for o in self.static] + \
               [(o, dyn_pos[k], True) for k, o in enumerate(self.dynamic)]
        for i, (o, (x, y), dyn) in enumerate(allo):
            m = Marker(header=h, ns="obstacles", id=i, action=Marker.ADD)
            m.type = Marker.CYLINDER if o["shape"] == "cylinder" else Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, -self.D / 2
            m.pose.orientation.w = 1.0
            if o["shape"] == "cylinder":
                m.scale.x = m.scale.y = 2 * o["radius"]
            else:
                m.scale.x, m.scale.y = o["size_x"], o["size_y"]
            m.scale.z = self.D
            m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.1, 0.1, 0.9) if dyn else (0.8, 0.4, 0.1, 0.9)
            m.text = o["name"]
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def step(self, _=None):
        now = rospy.Time.now()
        t = now.to_sec()
        dyn_pos = [position(o, t) for o in self.dynamic]
        for o, (x, y) in (zip(self.dynamic, dyn_pos) if self.spawned else []):
            s = ModelState(model_name=o["name"], reference_frame="world")
            s.pose.position.x, s.pose.position.y = self.gz(x, y)
            s.pose.orientation.w = 1.0
            self.state_pub.publish(s)
        cells = [self.fixed_cells] + [obstacle_cells(o, x, y) for o, (x, y) in zip(self.dynamic, dyn_pos)]
        self.cloud_pub.publish(cloud_msg(np.concatenate(cells), now))
        self.markers(now, dyn_pos)


def main():
    rospy.init_node("pool_scene")
    sc = Scene()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:   # wait for /clock
        rospy.sleep(0.1)
    sc.step()
    sc.spawn()
    rate = rospy.Rate(sc.rate)
    while not rospy.is_shutdown():
        sc.step()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
