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

#ifndef B3S_APPROXIMATE_MEAN_VOXEL_GRID_H_
#define B3S_APPROXIMATE_MEAN_VOXEL_GRID_H_

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>

#include <boost/unordered_map.hpp>

namespace pcl {

/** \brief Modified version of the PCL ApproximateVoxelGrid, that assembles a local 3D grid over a given PointCloud, and downsamples +
 * filters the data. In contrast to the original version, this filter calculates the real mean for each voxel cell and allows further
 * filtering with a threshold for the count. As a side effect, it guarantees to return only on cell per voxel and therefore further reduces
 * to number of output points.
 *
 * \author Joachim Clemens, original filter: James Bowman, Radu B. Rusu
 * \ingroup filters
 */
template<typename PointT>
class ApproximateMeanVoxelGrid : public pcl::Filter<PointT> {
    using pcl::Filter<PointT>::filter_name_;
    using pcl::Filter<PointT>::getClassName;
    using pcl::Filter<PointT>::input_;
    using pcl::Filter<PointT>::indices_;

    using PointCloud         = typename Filter<PointT>::PointCloud;
    using PointCloudPtr      = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

private:
    struct HistoryElement {
        HistoryElement() : count( 0 ), centroid() {}
        int             count;
        Eigen::VectorXf centroid;
    };

    struct vector_hash {
        size_t operator()( const Eigen::Vector3i &p ) const { return ( p[0] * 73856093 ) ^ ( p[1] * 19349663 ) ^ ( p[2] * 83492791 ); }
    };

public:
    using Ptr      = shared_ptr<ApproximateMeanVoxelGrid<PointT> >;
    using ConstPtr = shared_ptr<const ApproximateMeanVoxelGrid<PointT> >;


    /** \brief Empty constructor. */
    ApproximateMeanVoxelGrid() :
        pcl::Filter<PointT>(),
        leaf_size_( Eigen::Vector3f::Ones() ),
        inverse_leaf_size_( Eigen::Array3f::Ones() ),
        downsample_all_data_( true ),
        count_threshold_( 1 ),
        save_counts_( false )
    {
        filter_name_ = "ApproximateMeanVoxelGrid";
    }

    /** \brief Copy constructor.
     * \param[in] src the approximate voxel grid to copy into this.
     */
    ApproximateMeanVoxelGrid( const ApproximateMeanVoxelGrid &src ) :
        pcl::Filter<PointT>(),
        leaf_size_( src.leaf_size_ ),
        inverse_leaf_size_( src.inverse_leaf_size_ ),
        downsample_all_data_( src.downsample_all_data_ ),
        count_threshold_( src.count_threshold_ ),
        history_( src.history_ ),
        save_counts_( src.save_counts_ ),
        counts_( src.counts_ )
    {
        // Nothing to do here
    }


    /** \brief Destructor.
     */
    ~ApproximateMeanVoxelGrid()
    {
        // Nothing to do here
    }


    /** \brief Copy operator.
     * \param[in] src the approximate voxel grid to copy into this.
     */
    inline ApproximateMeanVoxelGrid &operator=( const ApproximateMeanVoxelGrid &src )
    {
        leaf_size_           = src.leaf_size_;
        inverse_leaf_size_   = src.inverse_leaf_size_;
        downsample_all_data_ = src.downsample_all_data_;
        count_threshold_     = src.count_threshold_;
        history_             = src.history_;
        save_counts_         = src.save_counts_;
        counts_              = src.counts_;
        return ( *this );
    }

    /** \brief Set the voxel grid leaf size.
     * \param[in] leaf_size the voxel grid leaf size
     */
    inline void setLeafSize( const Eigen::Vector3f &leaf_size )
    {
        leaf_size_         = leaf_size;
        inverse_leaf_size_ = Eigen::Array3f::Ones() / leaf_size_.array();
    }

    /** \brief Set the voxel grid leaf size.
     * \param[in] lx the leaf size for X
     * \param[in] ly the leaf size for Y
     * \param[in] lz the leaf size for Z
     */
    inline void setLeafSize( float lx, float ly, float lz ) { setLeafSize( Eigen::Vector3f( lx, ly, lz ) ); }

    /** \brief Get the voxel grid leaf size. */
    inline Eigen::Vector3f getLeafSize() const { return ( leaf_size_ ); }

    /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
     * \param downsample the new value (true/false)
     */
    inline void setDownsampleAllData( bool downsample ) { downsample_all_data_ = downsample; }

    /** \brief Get the state of the internal downsampling parameter (true if
     * all fields need to be downsampled, false if just XYZ).
     */
    inline bool getDownsampleAllData() const { return ( downsample_all_data_ ); }

    /** \brief All cells with a smaller count will be excluded in the output.
     * \param threshold the new value
     */
    inline void setCountThreshold( int threshold ) { count_threshold_ = threshold; }

    /** \brief Get the value of the internal count threshold (cells with a smaller
     * count will be excluded in the output).
     */
    inline int getCountThreshold() const { return ( count_threshold_ ); }

    /** \brief Save counts for each voxel
     */
    inline void setSaveCounts( bool save_counts ) { save_counts_ = save_counts; }

    /** \brief Whether the counts for each voxel are saved
     */
    inline bool getSaveCounts() const { return save_counts_; }

    /** \brief Get the counts for the voxels (value is undefinde,
     * if save_counts_ is false)
     */
    inline const std::vector<int> &getCounts() const { return counts_; }

protected:
    /** \brief The size of a leaf. */
    Eigen::Vector3f leaf_size_;

    /** \brief Compute 1/leaf_size_ to avoid division later */
    Eigen::Array3f inverse_leaf_size_;

    /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
    bool downsample_all_data_;

    /** \brief All cells with a smaller count will be excluded in the output. */
    int count_threshold_;

    /** \brief history buffer */
    boost::unordered_map<Eigen::Vector3i, HistoryElement, ApproximateMeanVoxelGrid::vector_hash> history_;

    /** \brief counts for each voxel (only set, if save_counts_ is true) */
    std::vector<int> counts_;

    /** \brief whether to save the counts for each voxel */
    bool save_counts_;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;

    /** \brief Downsample a Point Cloud using a voxelized grid approach
     * \param output the resultant point cloud message
     */
    void applyFilter( PointCloud &output );

    /** \brief Write a single point from the hash to the output cloud
     */
    inline void flush( PointCloud &output, size_t op, HistoryElement *hhe, int rgba_index, int centroid_size );
};

}  // namespace pcl


#include "ApproximateMeanVoxelGrid.hpp"

#endif  //#ifndef B3S_APPROXIMATE_MEAN_VOXEL_GRID_H_
