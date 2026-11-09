std_msgs/Header header

time traj_start_time

geometry_msgs/Vector3[] innerpoints
float64[] t_pts

geometry_msgs/Vector3 init_p
geometry_msgs/Vector3 init_v
geometry_msgs/Vector3 init_a
geometry_msgs/Vector3 tail_p
geometry_msgs/Vector3 tail_v
geometry_msgs/Vector3 tail_a

geometry_msgs/Vector3 start_position

geometry_msgs/Vector3 ICR
# Trajectory model. DIFF (default, original DDR-opt): flat output (yaw, s) in x/y of the vectors above.
# HOLONOMIC (robot:=rov_holo): flat output (x, y, theta) in x/y/z of init_*/tail_*/innerpoints, world frame.
uint8 DIFF=0
uint8 HOLONOMIC=1
uint8 model
