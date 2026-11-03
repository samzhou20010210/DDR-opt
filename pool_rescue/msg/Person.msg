# A person in the pool, expressed in the world frame (header.frame_id = "world").
std_msgs/Header header          # stamp = observation time
string id                       # unique, stable across messages (e.g. "p1")
geometry_msgs/Point position    # [m] footprint centre; z ignored (0)
geometry_msgs/Vector3 velocity  # [m/s] planar velocity in world frame; z ignored (0)
float64 yaw                     # [rad] orientation of the footprint's length axis in world frame
float64 length                  # [m] occupied footprint along yaw
float64 width                   # [m] occupied footprint across yaw
