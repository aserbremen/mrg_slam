#pragma once

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Dense>
#include <mrg_slam/keyframe.hpp>

namespace mrg_slam_pcl {

using PointType = mrg_slam::KeyFrame::PointT;

/**
 * @brief Fill the center of a point cloud with points on the ground plane. This is useful for initializing the SLAM algorithm for usage in
 * a navigation system.
 */
void fill_ground_plane( pcl::PointCloud<PointType>::Ptr cloud, double radius, double map_resolution );

}  // namespace mrg_slam_pcl