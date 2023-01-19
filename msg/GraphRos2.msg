# Graph

std_msgs/Header header

string robot_name

uint64 latest_keyframe_gid                  # GID of latest keyframe
geometry_msgs/Pose latest_keyframe_odom     # odometry transform of latest keyframe, used to calc odom to map transform for that robot

KeyFrameRos2[] keyframes
EdgeRos[] edges