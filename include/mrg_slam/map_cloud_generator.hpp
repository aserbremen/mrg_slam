// SPDX-License-Identifier: BSD-2-Clause

#ifndef MAP_CLOUD_GENERATOR_HPP
#define MAP_CLOUD_GENERATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mrg_slam/keyframe.hpp>
#include <vector>

namespace mrg_slam {

/**
 * @brief this class generates a map point cloud from registered keyframes
 */
class MapCloudGenerator {
public:
    using PointT = pcl::PointXYZI;

    MapCloudGenerator();
    ~MapCloudGenerator();

    /**
     * @brief generates a map point cloud
     * @param keyframes   snapshots of keyframes
     * @param resolution  voxel grid resolution of generated map
     * @param min_points_per_voxel minimum number of points per voxel to keep the corresponding points in the map
     * @return generated map point cloud
     */
    pcl::PointCloud<PointT>::Ptr generate( const std::vector<KeyFrameSnapshot::Ptr>& keyframes, float resolution, int min_points_per_voxel,
                                           float distance_far_thresh, bool skip_first_cloud = true ) const;
};

}  // namespace mrg_slam

#endif  // MAP_POINTCLOUD_GENERATOR_HPP
