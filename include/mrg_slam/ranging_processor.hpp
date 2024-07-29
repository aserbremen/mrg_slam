// SPDX-License-Identifier: BSD-2-Clause

#ifndef RANGING_PROCESSOR_HPP
#define RANGING_PROCESSOR_HPP

// C++
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
// ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_radio_ranging_interfaces/msg/range.hpp>
// mrg_slam
#include <mrg_slam/graph_slam.hpp>  // contains g2o types (forward declarations)
#include <mrg_slam/keyframe.hpp>
// Eigen
#include <Eigen/Dense>

namespace mrg_slam {
struct RangingData {
public:
    RangingData() : initialized( false ), pose( Eigen::Isometry3d::Identity() ), position_sub( nullptr ), se3_vertex( nullptr ) {}
    bool                                                     initialized;
    Eigen::Isometry3d                                        pose;
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr>      position_deque;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub;
    g2o::VertexSE3                                          *se3_vertex;
    std::vector<g2o::EdgeSE3Ranging *>                       edges_se3_ranging;
};

class RangingProcessor {
public:
    RangingProcessor();

    void onInit( rclcpp::Node::SharedPtr _node );

    void position_callback( const nav_msgs::msg::Odometry::SharedPtr position_msg );

    void ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

    void init_positions( std::shared_ptr<GraphSLAM> &graph_slam );

private:
    rclcpp::Node::SharedPtr node;

    std::string              own_name;
    std::vector<std::string> ranging_names;
    double                   ranging_position_stddev;
    double                   ranging_orientation_stddev;
    double                   ranging_max_time_diff;

    std::string ranging_position_topic;
    // ranging name -> all relevant data
    std::unordered_map<std::string, RangingData> ranging_data_map;

    std::string                                                                ranging_topic;
    rclcpp::Subscription<ros2_radio_ranging_interfaces::msg::Range>::SharedPtr ranging_sub;

    std::mutex                                                            range_queue_mutex;
    std::deque<ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr> range_queue;
};

}  // namespace mrg_slam

#endif  // RANGING_PROCESSOR_HPP