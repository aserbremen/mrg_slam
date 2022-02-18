// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

#include <hdl_graph_slam/global_id.hpp>


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
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;
  using ConstPtr = std::shared_ptr<const KeyFrame>;

  KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg = nullptr);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  void set_gid(const GlobalIdGenerator &gid_generator);

  void save(const std::string& directory);
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  long id() const;
  Eigen::Isometry3d estimate() const;

  bool edge_exists(const KeyFrame &other) const;

public:
  ros::Time stamp;                                // timestamp
  Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)
  double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  GlobalId gid;                                   // global id
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;   // point cloud ROS msg
  boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS

  boost::optional<Eigen::Vector3d> acceleration;    //
  boost::optional<Eigen::Quaterniond> orientation;  //

  g2o::VertexSE3* node;  // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(long id, const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg = nullptr);

  ~KeyFrameSnapshot();

public:
  long id;
  Eigen::Isometry3d pose;                         // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;   // point cloud ROS msg
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_HPP
