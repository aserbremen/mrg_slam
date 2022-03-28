# Edge of a graph

uint64 gid

uint64 from_gid
uint64 to_gid

uint8 type  # 0 = odom, 1 = loop

geometry_msgs/Pose relative_pose

# Row-major representation of the 6x6 information matrix
float64[36] information