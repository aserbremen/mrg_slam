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
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>

namespace mrg_slam {

class RangingProcessor {
public:
    RangingProcessor();

    void onInit( rclcpp::Node::SharedPtr _node );

    void position_callback( const nav_msgs::msg::Odometry::SharedPtr position_msg );

    void ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

    void init_positions( std::shared_ptr<GraphSLAM> &graph_slam );

private:
    std::shared_ptr<rclcpp::Node> node;

    std::string              own_name;
    std::vector<std::string> ranging_names;
    std::vector<std::string> multi_robot_names;

    std::string                                                                          position_sub_topic;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>                position_subs;
    std::unordered_map<std::string, std::deque<nav_msgs::msg::Odometry::ConstSharedPtr>> others_position_map;

    std::string                                                                ranging_robot_sub_topic;
    std::string                                                                ranging_sub_topic;
    rclcpp::Subscription<ros2_radio_ranging_interfaces::msg::Range>::SharedPtr ranging_sub;


    std::mutex                                                            ranging_queue_mutex;
    std::deque<ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr> ranging_queue;
};

}  // namespace mrg_slam

#endif  // RANGING_PROCESSOR_HPP