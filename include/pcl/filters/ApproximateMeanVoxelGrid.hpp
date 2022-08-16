/*
 * Software License Agreement (BSD License)
 *
 *  B3SLAM - Bernoulli, Beta, and Belief Function SLAM
 *  Copyright (c) 2013-2019, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is a modified version of the PCL ApproximateVoxelGrid filter
 *  Copyright (c) 2009, Willow Garage, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of B3SLAM nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef B3S_APPROXIMATE_MEAN_VOXEL_GRID_HPP_
#define B3S_APPROXIMATE_MEAN_VOXEL_GRID_HPP_

#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <boost/mpl/size.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::ApproximateMeanVoxelGrid<PointT>::flush( PointCloud &output, size_t op, HistoryElement *hhe, int rgba_index, int centroid_size )
{
    hhe->centroid /= static_cast<float>( hhe->count );
    pcl::for_each_type<FieldList>( xNdCopyEigenPointFunctor<PointT>( hhe->centroid, output.points[op] ) );
    // ---[ RGB special case
    if( rgba_index >= 0 ) {
        // pack r/g/b into rgb
        float r = hhe->centroid[centroid_size - 3], g = hhe->centroid[centroid_size - 2], b = hhe->centroid[centroid_size - 1];
        int   rgb = ( static_cast<int>( r ) ) << 16 | ( static_cast<int>( g ) ) << 8 | ( static_cast<int>( b ) );
        memcpy( reinterpret_cast<char *>( &output.points[op] ) + rgba_index, &rgb, sizeof( float ) );
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::ApproximateMeanVoxelGrid<PointT>::applyFilter( PointCloud &output )
{
    int centroid_size = 4;
    if( downsample_all_data_ ) centroid_size = boost::mpl::size<FieldList>::value;

    // ---[ RGB special case
    std::vector<pcl::PCLPointField> fields;
    int                             rgba_index = -1;
    rgba_index                                 = pcl::getFieldIndex<PointT>( "rgb", fields );
    if( rgba_index == -1 ) rgba_index = pcl::getFieldIndex<PointT>( "rgba", fields );
    if( rgba_index >= 0 ) {
        rgba_index = fields[rgba_index].offset;
        centroid_size += 3;
    }

    history_.clear();
    Eigen::VectorXf scratch( Eigen::VectorXf::Zero( centroid_size ) );

    for( size_t cp = 0; cp < input_->points.size(); ++cp ) {
        Eigen::Vector3i ixyz( static_cast<int>( floor( input_->points[cp].x * inverse_leaf_size_[0] ) ),
                              static_cast<int>( floor( input_->points[cp].y * inverse_leaf_size_[1] ) ),
                              static_cast<int>( floor( input_->points[cp].z * inverse_leaf_size_[2] ) ) );

        HistoryElement *hhe = &history_[ixyz];

        if( !hhe->count ) hhe->centroid = Eigen::VectorXf::Zero( centroid_size );
        hhe->count++;

        // Unpack the point into scratch, then accumulate
        // ---[ RGB special case
        if( rgba_index >= 0 ) {
            // fill r/g/b data
            pcl::RGB rgb;
            memcpy( &rgb, ( reinterpret_cast<const char *>( &input_->points[cp] ) ) + rgba_index, sizeof( pcl::RGB ) );
            scratch[centroid_size - 3] = rgb.r;
            scratch[centroid_size - 2] = rgb.g;
            scratch[centroid_size - 1] = rgb.b;
        }
        pcl::for_each_type<FieldList>( xNdCopyPointEigenFunctor<PointT>( input_->points[cp], scratch ) );
        hhe->centroid += scratch;
    }


    output.points.resize( history_.size() );
    if( save_counts_ ) {
        counts_.resize( history_.size() );
    }
    size_t op = 0;  // output pointer
    for( auto &hhe : history_ ) {
        if( hhe.second.count && hhe.second.count >= count_threshold_ ) {
            flush( output, op, &hhe.second, rgba_index, centroid_size );
            if( save_counts_ ) {
                counts_[op] = hhe.second.count;
            }
            op++;
        }
    }
    output.resize( op );
    output.width    = output.size();
    output.height   = 1;      // downsampling breaks the organized structure
    output.is_dense = false;  // we filter out invalid points
}

#define PCL_INSTANTIATE_ApproximateMeanVoxelGrid( T ) template class PCL_EXPORTS pcl::ApproximateMeanVoxelGrid<T>;

#endif  // B3S_APPROXIMATE_MEAN_VOXEL_GRID_HPP_
