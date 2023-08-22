// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <g2o/core/sparse_block_matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <hdl_graph_slam/edge.hpp>
#include <hdl_graph_slam/global_id.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointT   = pcl::PointXYZI;
    using Ptr      = std::shared_ptr<KeyFrame>;
    using ConstPtr = std::shared_ptr<const KeyFrame>;

    KeyFrame( const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance,
              const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg = nullptr );
    KeyFrame( const std::string& directory, g2o::HyperGraph* graph );
    virtual ~KeyFrame();

    void set_gid( const GlobalIdGenerator& gid_generator );

    Eigen::Matrix<double, 6, 6> covariance( const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals ) const;

    void save( const std::string& directory );
    bool load( const std::string& directory, g2o::HyperGraph* graph );

    long              id() const;
    Eigen::Isometry3d estimate() const;

    bool edge_exists( const KeyFrame& other, const rclcpp::Logger& logger ) const;

public:
    builtin_interfaces::msg::Time     stamp;                     // timestamp
    Eigen::Isometry3d                 odom;                      // odometry (estimated by scan_matching_odometry)
    double                            accum_distance;            // accumulated distance from the first node (by scan_matching_odometry)
    GlobalId                          gid;                       // global id
    bool                              exclude_from_map;          // whether the corresponding point cloud should be excluded from the map
    pcl::PointCloud<PointT>::ConstPtr cloud;                     // point cloud
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;     // point cloud ROS msg
    boost::optional<Eigen::Vector4d>              floor_coeffs;  // detected floor's coefficients
    boost::optional<Eigen::Vector3d>              utm_coord;     // UTM coord obtained by GPS

    boost::optional<Eigen::Vector3d>    acceleration;  //
    boost::optional<Eigen::Quaterniond> orientation;   //

    g2o::VertexSE3* node;  // node instance

    Edge::Ptr prev_edge = nullptr;  // previous edge TYPE_ODOM
    Edge::Ptr next_edge = nullptr;  // next edge TYPE_ODOM
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using PointT = KeyFrame::PointT;
    using Ptr    = std::shared_ptr<KeyFrameSnapshot>;

    KeyFrameSnapshot( const KeyFrame::Ptr& key, const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals = nullptr );
    /*
    KeyFrameSnapshot( long id, const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud, bool exclude_from_map = false,
                      const Eigen::MatrixXd* covariance = nullptr );
                      */

    ~KeyFrameSnapshot();

public:
    builtin_interfaces::msg::Time     stamp;  // timestamp
    long                              id;
    GlobalId                          gid;               // global id
    Eigen::Isometry3d                 pose;              // pose estimated by graph optimization
    pcl::PointCloud<PointT>::ConstPtr cloud;             // point cloud
    bool                              exclude_from_map;  // whether the corresponding point cloud should be excluded from the map
    Eigen::MatrixXd                   covariance;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_HPP
