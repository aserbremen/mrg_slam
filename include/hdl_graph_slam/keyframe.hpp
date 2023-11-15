// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <g2o/core/sparse_block_matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp>
#include <boost/uuid/uuid.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

// forward declaration for circular dependency
class Edge;

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointT   = pcl::PointXYZI;
    using Ptr      = std::shared_ptr<KeyFrame>;
    using ConstPtr = std::shared_ptr<const KeyFrame>;

    KeyFrame( const std::string& robot_name, const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom,
              int odom_keyframe_counter, const boost::uuids::uuid& uuid, double accum_distance,
              const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg = nullptr );
    KeyFrame( const std::string& directory, g2o::HyperGraph* graph );
    virtual ~KeyFrame();

    Eigen::Matrix<double, 6, 6> covariance( const std::shared_ptr<g2o::SparseBlockMatrixX>& marginals ) const;

    void save( const std::string& directory );
    bool load( const std::string& directory, g2o::HyperGraph* graph );

    long              id() const;
    Eigen::Isometry3d estimate() const;

    bool edge_exists( const KeyFrame& other, const rclcpp::Logger& logger ) const;

public:
    std::string                   robot_name;             // robot name
    builtin_interfaces::msg::Time stamp;                  // timestamp
    Eigen::Isometry3d             odom;                   // odometry (estimated by scan_matching_odometry)
    int                           odom_keyframe_counter;  // odometry keyframe counter for each robot
    boost::uuids::uuid            uuid;                   // unique id
    std::string                   uuid_str;               // unique id string
    double                        accum_distance;         // accumulated distance from the first node (by scan_matching_odometry)
    bool                          first_keyframe;  // first keyframe of slam, the corresponding point cloud should be excluded from the map
    pcl::PointCloud<PointT>::ConstPtr             cloud;         // point cloud
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;     // point cloud ROS msg
    boost::optional<Eigen::Vector4d>              floor_coeffs;  // detected floor's coefficients
    boost::optional<Eigen::Vector3d>              utm_coord;     // UTM coord obtained by GPS

    boost::optional<Eigen::Vector3d>    acceleration;  //
    boost::optional<Eigen::Quaterniond> orientation;   //

    g2o::VertexSE3* node;  // node instance

    // Only for keyframes starting from the first keyframe. First keyframe has no previous edge.
    // Anchor keyframe has no prev or next edge per our definition.
    std::shared_ptr<Edge> prev_edge = nullptr;  // previous edge TYPE_ODOM
    std::shared_ptr<Edge> next_edge = nullptr;  // next edge TYPE_ODOM

    std::string readable_id;
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
    KeyFrameSnapshot( long id, const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud, bool first_keyframe = false,
                      const Eigen::MatrixXd* covariance = nullptr );
                      */

    ~KeyFrameSnapshot();

public:
    // TODO find out whether to use uuid or uuid_str
    builtin_interfaces::msg::Time     stamp;  // timestamp
    long                              id;
    boost::uuids::uuid                uuid;   // global id
    Eigen::Isometry3d                 pose;   // pose estimated by graph optimization
    pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
    bool            first_keyframe;           // first keyframe of slam, the corresponding point cloud should be excluded from the map
    Eigen::MatrixXd covariance;
};

}  // namespace hdl_graph_slam

// Include edge after keyframe to avoid circular dependency compilation error
#include <hdl_graph_slam/edge.hpp>


#endif  // KEYFRAME_HPP
