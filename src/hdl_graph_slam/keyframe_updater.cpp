// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/keyframe_updater.hpp>

namespace hdl_graph_slam {

KeyframeUpdater::KeyframeUpdater(ros::NodeHandle& pnh) : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
  keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 2.0);
  keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 2.0);

  accum_distance = 0.0;
}


bool KeyframeUpdater::update(const Eigen::Isometry3d& pose) {
  // first frame is always registered to the graph
  if(is_first) {
    is_first = false;
    prev_keypose = pose;
    return true;
  }

  // calculate the delta transformation from the previous keyframe
  Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
  double dx = delta.translation().norm();
  double da = Eigen::AngleAxisd(delta.linear()).angle();

  // too close to the previous frame
  if(dx < keyframe_delta_trans && da < keyframe_delta_angle) {
    return false;
  }

  accum_distance += dx;
  prev_keypose = pose;
  return true;
}


double KeyframeUpdater::get_accum_distance() const {
  return accum_distance;
}


}  // namespace hdl_graph_slam
