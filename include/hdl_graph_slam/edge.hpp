// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_HPP
#define EDGE_HPP

#include <ros/ros.h>
#include <Eigen/Dense>


namespace g2o {
class EdgeSE3;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct Edge {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Edge>;

  Edge(const g2o::EdgeSE3* edge);
  //Edge(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~Edge();

  long id() const;
  long from_id() const;
  long to_id() const;
  const Eigen::Isometry3d& relative_pose() const;
  const Eigen::Matrix<double, 6, 6>& information() const;

public:
  const g2o::EdgeSE3*   edge;       // edge instance
};

#if 0

/**
 * @brief EdgeSnapshot for publishing graph
 */
struct EdgeSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  EdgeSnapshot(const Edge::Ptr& edge);
  //EdgeSnapshot(const g2o::EdgeSE3* edge, ...);

  ~EdgeSnapshot();

public:
  Eigen::Isometry3d pose;                         // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;   // point cloud ROS msg
};

#endif

}  // namespace hdl_graph_slam

#endif  // EDGE_HPP
