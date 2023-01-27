// SPDX-License-Identifier: BSD-2-Clause

#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <pcl/registration/registration.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

namespace hdl_graph_slam {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
// pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method( ros::NodeHandle& pnh );

/**
 * @brief
 * @param node Shared pointer to main node
 * @return pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
 */
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method( rclcpp::Node::SharedPtr node );

}  // namespace hdl_graph_slam

#endif  //
