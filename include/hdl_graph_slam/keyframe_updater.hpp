// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

namespace hdl_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(ros::NodeHandle& pnh);

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose);

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const;

private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
