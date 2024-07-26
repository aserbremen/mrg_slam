// SPDX-License-Identifier: BSD-2-Clause

#ifndef RANGING_PROCESSOR_HPP
#define RANGING_PROCESSOR_HPP

// C++
#include <deque>
#include <memory>
#include <mutex>
#include <string>
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

    void ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes );

private:
    std::shared_ptr<rclcpp::Node> node;

    std::string own_name;

    std::string                                                                ranging_sub_topic;
    std::vector<std::string>                                                   ranging_names;
    rclcpp::Subscription<ros2_radio_ranging_interfaces::msg::Range>::SharedPtr ranging_sub;

    std::mutex                                                            ranging_queue_mutex;
    std::deque<ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr> ranging_queue;
};

}  // namespace mrg_slam

#endif  // RANGING_PROCESSOR_HPP