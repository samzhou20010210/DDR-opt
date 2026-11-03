#!/usr/bin/env python3
"""pool_env: static 2D pool environment generated from the central config (/pool/...).

Publishes (latched):
  /pool/env/boundary  geometry_msgs/PolygonStamped  water area [0, L] x [0, W] in the world frame
  /pool/env/markers   visualization_msgs/MarkerArray water surface, walls, standby pose, extraction zone, labels
"""
import math

import rospy
from geometry_msgs.msg import Point, Point32, PolygonStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def p(path):
    return rospy.get_param("/pool/" + path.replace(".", "/") + "/value")


def marker(ns, mid, mtype, frame, rgba):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


def rect_strip(cx, cy, yaw, length, width, z):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for bx, by in ((length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2),
                   (-length / 2, width / 2), (length / 2, width / 2)):
        pts.append(Point(cx + c * bx - s * by, cy + s * bx + c * by, z))
    return pts


def main():
    rospy.init_node("pool_env")
    world = p("frames.world")
    L, W = p("pool.length"), p("pool.width")
    t = p("pool.wall_thickness_in_map")
    sx, sy, syaw = p("standby.x"), p("standby.y"), p("standby.yaw")
    vl, vw = p("vessel.length"), p("vessel.width")

    boundary = PolygonStamped()
    boundary.header.frame_id = world
    boundary.polygon.points = [Point32(0, 0, 0), Point32(L, 0, 0), Point32(L, W, 0), Point32(0, W, 0)]

    arr = MarkerArray()
    water = marker("water", 0, Marker.CUBE, world, (0.10, 0.45, 0.85, 0.35))
    water.pose.position = Point(L / 2, W / 2, -0.01)
    water.scale.x, water.scale.y, water.scale.z = L, W, 0.02
    arr.markers.append(water)

    # walls drawn just outside the water edge so the inner wall face is exactly the boundary
    walls = [(-t / 2, W / 2, t, W + 2 * t), (L + t / 2, W / 2, t, W + 2 * t),
             (L / 2, -t / 2, L, t), (L / 2, W + t / 2, L, t)]
    for i, (cx, cy, ax, ay) in enumerate(walls):
        w = marker("walls", i, Marker.CUBE, world, (0.6, 0.6, 0.6, 1.0))
        w.pose.position = Point(cx, cy, 0.15)
        w.scale.x, w.scale.y, w.scale.z = ax, ay, 0.3
        arr.markers.append(w)

    edge = marker("boundary", 0, Marker.LINE_STRIP, world, (1.0, 1.0, 1.0, 1.0))
    edge.scale.x = 0.03
    edge.points = rect_strip(L / 2, W / 2, 0.0, L, W, 0.02)
    arr.markers.append(edge)

    # lane ropes (World Aquatics layout from config): visual only, NOT obstacles — the vessel passes over them
    n_lanes, lane_w = int(p("pool.lanes.count")), p("pool.lanes.lane_width")
    outer, rope_d = p("pool.lanes.outer_space"), p("pool.lanes.rope_diameter")
    ropes = [outer + i * lane_w for i in range(1, n_lanes)]
    lane = marker("lane_ropes", 0, Marker.LINE_LIST, world, (1.0, 1.0, 1.0, 1.0))
    lane.scale.x = rope_d
    for y in ropes:                      # alternating red/white 1 m float sections
        x, k = 0.0, 0
        while x < L - 1e-9:
            x2 = min(x + 1.0, L)
            col = (0.9, 0.1, 0.1, 1.0) if k % 2 == 0 else (1.0, 1.0, 1.0, 1.0)
            lane.points += [Point(x, y, 0.03), Point(x2, y, 0.03)]
            lane.colors += [ColorRGBA(*col), ColorRGBA(*col)]
            x, k = x2, k + 1
    arr.markers.append(lane)

    # static obstacles of the active scenario (pool_nav.launch sets /pool/scenario)
    scenario = rospy.get_param("/pool/scenario", "")
    obstacles = rospy.get_param("/pool/scenarios/%s/obstacles" % scenario, []) if scenario else []
    for i, ob in enumerate(obstacles):
        cx, cy, ax, ay, yaw = ob["box"]
        o = marker("obstacles", i, Marker.CUBE, world, (0.55, 0.15, 0.1, 0.9))
        o.pose.position = Point(cx, cy, 0.2)
        o.pose.orientation.z, o.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.scale.x, o.scale.y, o.scale.z = ax, ay, 0.4
        arr.markers.append(o)

    standby = marker("standby", 0, Marker.LINE_STRIP, world, (1.0, 1.0, 0.0, 0.8))
    standby.scale.x = 0.02
    standby.points = rect_strip(sx, sy, syaw, vl, vw, 0.02)
    arr.markers.append(standby)

    ez = marker("extraction_zone", 0, Marker.LINE_STRIP, world, (0.1, 0.9, 0.2, 1.0))
    ez.scale.x = 0.04
    ez.points = rect_strip(p("extraction_zone.x"), p("extraction_zone.y"), p("extraction_zone.theta"),
                           p("extraction_zone.length"), p("extraction_zone.width"), 0.02)
    arr.markers.append(ez)

    for i, (text, x, y) in enumerate((("standby", sx, sy - 0.8),
                                      ("extraction: " + str(p("extraction_zone.name")),
                                       p("extraction_zone.x") + 1.6, p("extraction_zone.y")),
                                      ("pool %.1f x %.1f m" % (L, W), L / 2, W + 0.8))):
        lbl = marker("labels", i, Marker.TEXT_VIEW_FACING, world, (1.0, 1.0, 1.0, 1.0))
        lbl.pose.position = Point(x, y, 0.3)
        lbl.scale.z = 0.4
        lbl.text = text
        arr.markers.append(lbl)

    pub_b = rospy.Publisher("/pool/env/boundary", PolygonStamped, queue_size=1, latch=True)
    pub_m = rospy.Publisher("/pool/env/markers", MarkerArray, queue_size=1, latch=True)
    stamp = rospy.Time.now()
    boundary.header.stamp = stamp
    for m in arr.markers:
        m.header.stamp = stamp
    pub_b.publish(boundary)
    pub_m.publish(arr)
    rospy.loginfo("[pool_env] pool %.1f x %.1f m, %d lane ropes at y=%s (not obstacles), scenario %r with %d obstacles, %d markers",
                  L, W, len(ropes), ropes, scenario, len(obstacles), len(arr.markers))
    rospy.spin()


if __name__ == "__main__":
    main()
