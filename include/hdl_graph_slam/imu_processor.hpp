// SPDX-License-Identifier: BSD-2-Clause

#ifndef IMU_PROCESSOR_HPP
#define IMU_PROCESSOR_HPP

// ROS2 migration
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <tf/transform_listener.h>

#include <deque>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <mutex>


namespace hdl_graph_slam {

class ImuProcessor {
public:
    ImuProcessor() {}

    // void onInit( ros::NodeHandle &nh, ros::NodeHandle &mt_nh, ros::NodeHandle &private_nh );
    void onInit( rclcpp::Node::SharedPtr &_node );

    void imu_callback( const sensor_msgs::msg::Imu::SharedPtr imu_msg );

    bool flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes, const std::string &base_frame_id );

private:
    rclcpp::Node::SharedPtr                                node;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    // ros::NodeHandle *private_nh;
    // ros::Subscriber imu_sub;

    // https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    std::unique_ptr<tf2_ros::Buffer>            tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

    double                                      imu_time_offset;
    bool                                        enable_imu_orientation;
    double                                      imu_orientation_edge_stddev;
    bool                                        enable_imu_acceleration;
    double                                      imu_acceleration_edge_stddev;
    std::mutex                                  imu_queue_mutex;
    std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_queue;
};

}  // namespace hdl_graph_slam

#endif  // IMU_PROCESSOR_HPP