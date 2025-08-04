// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief constructor
     * @param sharedptr to the node
     */
    KeyframeUpdater( rclcpp::Node::SharedPtr node );

    /**
     * @brief decide if a new frame should be registered to the graph
     * @param pose  pose of the frame
     * @return  if true, the frame should be registered
     */
    bool update( const Eigen::Isometry3d& pose );

    /**
     * @brief the last keyframe's accumulated distance from the first keyframe
     * @return accumulated distance
     */
    double get_accum_distance() const;

private:
    rclcpp::Node::SharedPtr node_;

    bool              is_first_;
    double            accum_distance_;
    Eigen::Isometry3d prev_keypose_;
};

}  // namespace mrg_slam

#endif  // KEYFRAME_UPDATOR_HPP
