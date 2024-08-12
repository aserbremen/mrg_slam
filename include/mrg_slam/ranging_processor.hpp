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

// Ranging data for a single ranging device
struct RangingData {
public:
    RangingData() : initialized( false ), position_sub( nullptr ), pose( Eigen::Isometry3d::Identity() ), se3_vertex( nullptr ) {}
    bool                                                     initialized;     // whether the ranging device has been initialized
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub;    // subscription to the position (odom) of the ranging device
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr>      position_deque;  // odom messages of the ranging device
    Eigen::Isometry3d                                        pose;            // pose of the ranging device
    g2o::VertexSE3                                          *se3_vertex;      // for now stationary vertex of the ranging device
    std::vector<g2o::EdgeSE3Ranging *>                       edges_se3_ranging;  // g2o ranging edges, can be used for visualization
};

class RangingProcessor {
public:
    RangingProcessor();

    void onInit( rclcpp::Node::SharedPtr _node );

    void position_callback( nav_msgs::msg::Odometry::ConstSharedPtr position_msg );

    void ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

    void init_positions( std::shared_ptr<GraphSLAM> &graph_slam );

    KeyFrame::Ptr find_closest_keyframe( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr range_msg,
                                         const std::vector<KeyFrame::Ptr>                         &keyframes );

    void add_ranging_edge( std::shared_ptr<GraphSLAM> &graph_slam, ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr range_msg,
                           KeyFrame::Ptr keyframe );

private:
    rclcpp::Node::SharedPtr node;

    std::string              own_name;
    std::vector<std::string> ranging_names;
    double                   ranging_position_stddev;
    double                   ranging_orientation_stddev;
    double                   ranging_max_time_diff;
    std::string              ranging_edge_robust_kernel;
    double                   ranging_edge_robust_kernel_size;
    double                   ranging_edge_extra_stddev;

    std::string ranging_position_topic;
    // ranging device name -> all relevant data
    std::unordered_map<std::string, RangingData> ranging_data_map;

    std::string                                                                ranging_topic;
    rclcpp::Subscription<ros2_radio_ranging_interfaces::msg::Range>::SharedPtr ranging_sub;

    std::mutex                                                            range_queue_mutex;
    std::deque<ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr> range_queue;
};

}  // namespace mrg_slam

#endif  // RANGING_PROCESSOR_HPP