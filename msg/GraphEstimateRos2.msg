# Graph estimate, in contrast to GraphRos, the transforamtion and uncertainty of the edges is marginalized into the keyframes

std_msgs/Header header

string robot_name

uint64 latest_keyframe_gid                  # GID of latest keyframe

KeyFrameEstimateRos2[] keyframes
EdgeEstimate[] edges