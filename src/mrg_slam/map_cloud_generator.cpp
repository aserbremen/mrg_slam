// SPDX-License-Identifier: BSD-2-Clause

#include <pcl/filters/ApproximateMeanVoxelGrid.h>

#include <mrg_slam/map_cloud_generator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrg_slam {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr
MapCloudGenerator::generate( const std::vector<KeyFrameSnapshot::Ptr>& keyframes, float resolution, int min_points_per_voxel,
                             float distance_far_thresh, bool skip_first_cloud ) const
{
    auto logger = rclcpp::get_logger( "MapCloudGenerator" );
    if( keyframes.empty() ) {
        RCLCPP_WARN( logger, "Keyframes are empty, cannot generate map cloud." );
        return nullptr;
    }

    pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
    cloud->reserve( keyframes.front()->cloud->size() * keyframes.size() );

    bool  use_distance_filter    = distance_far_thresh > 0;
    float distance_far_thresh_sq = distance_far_thresh * distance_far_thresh;

    for( const auto& keyframe : keyframes ) {
        // Exclude points from the first keyframe, since they might include points of rovers that have not been filtered
        if( keyframe->first_keyframe && skip_first_cloud ) {
            continue;
        }
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();

        if( use_distance_filter ) {
            for( const auto& src_pt : keyframe->cloud->points ) {
                if( src_pt.getVector3fMap().squaredNorm() > distance_far_thresh_sq ) {
                    continue;  // Skip points that are too far away
                }
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity        = src_pt.intensity;
                cloud->push_back( dst_pt );
            }
        } else {
            for( const auto& src_pt : keyframe->cloud->points ) {
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity        = src_pt.intensity;
                cloud->push_back( dst_pt );
            }
        }
    }

    if( cloud->empty() && keyframes.size() > 1 ) {
        RCLCPP_WARN( logger, "Cloud is empty after processing keyframes." );
        return nullptr;
    }

    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = false;

    if( resolution <= 0.0 ) {
        RCLCPP_INFO_STREAM( logger, "Generated map (skip first cloud: " << skip_first_cloud << ") with full resolution and size "
                                                                        << cloud->size() );
        return cloud;  // To get unfiltered point cloud with intensity
    }

    pcl::ApproximateMeanVoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud( cloud );
    voxelGridFilter.setLeafSize( resolution, resolution, resolution );
    voxelGridFilter.setCountThreshold( min_points_per_voxel );

    pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
    voxelGridFilter.filter( *filtered );

    RCLCPP_INFO_STREAM( logger, "Generated map (skip first cloud: " << skip_first_cloud << ") with resolution " << resolution
                                                                    << ", min points per voxel: " << min_points_per_voxel
                                                                    << ", distance far thresh " << distance_far_thresh << " and size "
                                                                    << filtered->size() );

    return filtered;
}

}  // namespace mrg_slam
