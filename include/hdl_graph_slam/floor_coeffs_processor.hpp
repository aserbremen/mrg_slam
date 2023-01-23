// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_COEFFS_PROCESSOR_HPP
#define FLOOR_COEFFS_PROCESSOR_HPP

// #include <hdl_graph_slam/FloorCoeffs.h>

#include <deque>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>
#include <mutex>

// ROS2 migration
#include <builtin_interfaces/msg/time.hpp>
#include <hdl_graph_slam/msg/floor_coeffs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hdl_graph_slam {

class FloorCoeffsProcessor {
public:
    FloorCoeffsProcessor() : private_nh( nullptr ) {}

    //
    void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );

    void floor_coeffs_callback( const hdl_graph_slam::FloorCoeffsConstPtr &floor_coeffs_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes,
                const std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> &keyframe_hash, const ros::Time &latest_keyframe_stamp );

    const g2o::VertexPlane *floor_plane_node() const { return floor_plane_node_ptr; }

private:
    ros::NodeHandle *private_nh;
    ros::Subscriber  floor_sub;

    double                                          floor_edge_stddev;
    std::mutex                                      floor_coeffs_queue_mutex;
    std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

    g2o::VertexPlane *floor_plane_node_ptr;
};

}  // namespace hdl_graph_slam

#endif  // FLOOR_COEFFS_PROCESSOR_HPP