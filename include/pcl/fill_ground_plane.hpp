/*
Copyright (C) 2017 S. Kasperski
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are
met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
software without specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Dense>
#include <mrg_slam/keyframe.hpp>

namespace mrg_slam_pcl {

using PointT = mrg_slam::KeyFrame::PointT;

/**
 * @brief Add a circular patch of points to the cloud by fitting a ground plane to the provided cloud using RANSAC. This is useful for
 * initializing the SLAM cloud for usage in a navigation system (points below the robot). If the robot is surrounded by walls or other
 * obstacles, a circular patch of points might be added to the wall. In such a case, use fill_ground_plane_simple() instead.
 */
void fill_ground_plane_ransac( pcl::PointCloud<PointT>::Ptr cloud, double radius, double map_resolution );


/**
 * @brief Add a circular patch of points to the cloud, pointing upwards, using the provided base_link pose. This is useful for
 * initializing the SLAM cloud for usage in a navigation system (points below the robot).
 */
void fill_ground_plane_simple( pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Isometry3d& base_link_pose, double radius,
                               double map_resolution );

/**
 * @brief Add a circular patch of points to the cloud according to the provided plane, normal, radius, and map resolution.
 */
void fill_cloud( pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Hyperplane<double, 3>& plane, const Eigen::Vector3d& normal,
                 double radius, double map_resolution );


}  // namespace mrg_slam_pcl