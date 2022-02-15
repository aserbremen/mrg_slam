// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_HPP
#define EDGE_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

#include <hdl_graph_slam/global_id.hpp>


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

  Edge(const g2o::EdgeSE3* edge, const GlobalIdGenerator &gid_generator);
  //Edge(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~Edge();

  long id() const;
  long from_id() const;
  long to_id() const;
  const Eigen::Isometry3d& relative_pose() const;
  const Eigen::Matrix<double, 6, 6>& information() const;

public:
  const g2o::EdgeSE3*   edge;       // edge instance
  GlobalId gid;
  GlobalId from_gid;
  GlobalId to_gid;
};

/**
 * @brief EdgeSnapshot for publishing graph
 */
struct EdgeSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<EdgeSnapshot>;

  EdgeSnapshot(const Edge::Ptr& edge);
  //EdgeSnapshot(...);

  ~EdgeSnapshot();

public:
  long                        id;
  long                        from_id;
  long                        to_id;
  Eigen::Isometry3d           relative_pose;
  Eigen::Matrix<double, 6, 6> information;
};

}  // namespace hdl_graph_slam

#endif  // EDGE_HPP
