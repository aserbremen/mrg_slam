// SPDX-License-Identifier: BSD-2-Clause

#ifndef IMU_PROCESSOR_HPP
#define IMU_PROCESSOR_HPP

// ROS2 migration
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace mrg_slam {

class ImuProcessor {
public:
    ImuProcessor() {}

    void onInit( rclcpp::Node::SharedPtr node );

    void imu_callback( const sensor_msgs::msg::Imu::SharedPtr imu_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes, const std::string &base_frame_id );

private:
    rclcpp::Node::SharedPtr                                node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::unique_ptr<tf2_ros::Buffer>            tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::mutex                                        imu_queue_mutex_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;
};

}  // namespace mrg_slam

#endif  // IMU_PROCESSOR_HPP