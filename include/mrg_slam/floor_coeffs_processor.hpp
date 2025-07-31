// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_COEFFS_PROCESSOR_HPP
#define FLOOR_COEFFS_PROCESSOR_HPP

#include <deque>
#include <mutex>
// mrg_slam
#include <mrg_slam/graph_database.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/ros_time_hash.hpp>
// ROS2
#include <builtin_interfaces/msg/time.hpp>
#include <mrg_slam_msgs/msg/floor_coeffs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

class FloorCoeffsProcessor {
public:
    FloorCoeffsProcessor() {}

    void onInit( rclcpp::Node::SharedPtr node );

    /**
     * @brief received floor coefficients are added to #floor_coeffs_queue
     * @param floor_coeffs_msg
     */
    void floor_coeffs_callback( mrg_slam_msgs::msg::FloorCoeffs::ConstSharedPtr floor_coeffs_msg );


    /**
     * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose
     * graph
     * @return if true, at least one floor plane edge is added to the pose graph
     */
    bool flush( std::shared_ptr<GraphDatabase> graph_db, std::shared_ptr<GraphSLAM> &graph_slam );

    const g2o::VertexPlane *floor_plane_node() const { return floor_plane_node_ptr_; }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<mrg_slam_msgs::msg::FloorCoeffs>::SharedPtr floor_sub_;

    std::mutex                                                  floor_coeffs_queue_mutex_;
    std::deque<mrg_slam_msgs::msg::FloorCoeffs::ConstSharedPtr> floor_coeffs_queue_;

    g2o::VertexPlane *floor_plane_node_ptr_;
};

}  // namespace mrg_slam

#endif  // FLOOR_COEFFS_PROCESSOR_HPP