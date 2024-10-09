// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <tf2/transform_datatypes.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 *
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
geometry_msgs::msg::TransformStamped matrix2transform( const rclcpp::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id,
                                                       const std::string& child_frame_id );


Eigen::Isometry3d pose2isometry( const geometry_msgs::msg::Pose& pose );


Eigen::Isometry3d tf2isometry( const geometry_msgs::msg::TransformStamped& trans );


geometry_msgs::msg::Pose isometry2pose( const Eigen::Isometry3d& mat );


Eigen::Isometry3d odom2isometry( const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg );


/**
 * @brief Prints ROS2 parameters for a given node.
 * 
 * @param param_interface
 * @param logger
 */
void print_ros2_parameters( rclcpp::node_interfaces::NodeParametersInterface::ConstSharedPtr param_interface,
                            const rclcpp::Logger&                                            logger );

}  // namespace mrg_slam

#endif  // ROS_UTILS_HPP
