#!/usr/bin/env python3
"""Obstacle map of the Gazebo pool for DDR-opt, in the pool-corner frame ("world").

Publishes a latched PointCloud2 ~cloud in the format of the 2D simulator's /global_map/laser_gridmap (one column of
points per occupied 0.1 m cell, z in [-0.5, 0.4]), which laser_simulator ray-casts into the planner's local point
cloud exactly as in the 2D runs.

Version 1: the 4 pool walls only (the pool props are not spawned by planner_gazebo.launch). Walls are `thickness`
deep outside the water, so the free water is exactly x in [0, size_x], y in [0, size_y].
TODO(dynamic obstacles): add boxes / cylinders from /gazebo/model_states and republish live (laser_simulator reads
its map once today, so it needs a re-read option too).
"""
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def main():
    rospy.init_node("pool_obstacles")
    sx, sy = rospy.get_param("~pool_size", [25.0, 50.0])
    res = rospy.get_param("~resolution", 0.1)
    th = rospy.get_param("~thickness", 0.3)
    xs = np.arange(-th + res / 2, sx + th, res)
    ys = np.arange(-th + res / 2, sy + th, res)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    wall = (X < 0) | (X > sx) | (Y < 0) | (Y > sy)
    cells = np.stack([X[wall], Y[wall]], axis=1)
    zs = np.arange(-5, 5) * res
    pts = [(float(x), float(y), float(z)) for x, y in cells for z in zs]
    h = Header(frame_id="world")
    pub = rospy.Publisher("~cloud", PointCloud2, queue_size=1, latch=True)
    rospy.loginfo("[pool_obstacles] pool %.1f x %.1f m (origin at the corner), walls only: %d cells, %d points",
                  sx, sy, len(cells), len(pts))
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        h.stamp = rospy.Time.now()
        pub.publish(pc2.create_cloud_xyz32(h, pts))
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
