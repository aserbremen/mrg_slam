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
     * @param resolution  resolution of generated map
     * @return generated map point cloud
     */
    pcl::PointCloud<PointT>::Ptr generate( const std::vector<KeyFrameSnapshot::Ptr>& keyframes, float resolution, int count_threshold,
                                           bool skip_first_cloud = true ) const;
};

}  // namespace mrg_slam

#endif  // MAP_POINTCLOUD_GENERATOR_HPP
