// SPDX-License-Identifier: BSD-2-Clause

// #include <pcl/octree/octree_search.h>
#include <pcl/filters/ApproximateMeanVoxelGrid.h>

#include <mrg_slam/map_cloud_generator.hpp>

namespace mrg_slam {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr
MapCloudGenerator::generate( const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution, int count_threshold ) const
{
    if( keyframes.empty() ) {
        std::cerr << "warning: keyframes empty!!" << std::endl;
        return nullptr;
    }

    pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
    cloud->reserve( keyframes.front()->cloud->size() * keyframes.size() );

    for( const auto& keyframe : keyframes ) {
        // Exclude points from the first keyframe, since they might include points of rovers that have not been filtered
        if( keyframe->first_keyframe ) {
            continue;
        }
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
        for( const auto& src_pt : keyframe->cloud->points ) {
            PointT dst_pt;
            dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
            dst_pt.intensity        = src_pt.intensity;
            cloud->push_back( dst_pt );
        }
    }

    if( cloud->empty() && keyframes.size() > 1 ) {
        std::cerr << "warning: cloud empty" << std::endl;
        return nullptr;
    }

    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = false;

    if( resolution <= 0.0 ) {
        return cloud;  // To get unfiltered point cloud with intensity
    }

    /*
    pcl::octree::OctreePointCloud<PointT> octree( resolution );
    octree.setInputCloud( cloud );
    octree.addPointsFromInputCloud();

    pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
    octree.getOccupiedVoxelCenters( filtered->points );

    filtered->width    = filtered->size();
    filtered->height   = 1;
    filtered->is_dense = false;
    */

    pcl::ApproximateMeanVoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud( cloud );
    voxelGridFilter.setLeafSize( resolution, resolution, resolution );
    voxelGridFilter.setCountThreshold( count_threshold );

    pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );
    voxelGridFilter.filter( *filtered );

    std::cout << "Generating map with resolution " << resolution << " and size " << filtered->size() << std::endl;

    return filtered;
}

}  // namespace mrg_slam
