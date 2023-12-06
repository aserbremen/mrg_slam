// SPDX-License-Identifier: BSD-2-Clause

#ifndef MRG_SLAM_REGISTRATIONS_HPP
#define MRG_SLAM_REGISTRATIONS_HPP

#include <pcl/registration/registration.h>

#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

/**
 * @brief select a pcl scan matching algorithm according to rosparams
 * @param node Shared pointer to main node
 * @return pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
 * There is no elegant way to pass a node shared ptr from within a rclcpp node class constructor (i.e. scan_matching_odometry_component.cpp)
 * to select the registration method. Thats why we pass a raw pointer of the base class rclcpp::Node to select_registration_method which
 * should be safe because the base class's constructor rclcpp::Node should be finalized.
 */
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method( rclcpp::Node* node_raw_ptr );

}  // namespace mrg_slam

#endif  //
