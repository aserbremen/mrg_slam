// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_HPP
#define EDGE_HPP

#include <Eigen/Dense>
#include <hdl_graph_slam/global_id.hpp>
// ROS2 migration
// ASTODO: Check whether ROS is needed at all in this class
// #include <rclcpp/rclcpp.hpp>


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

    enum Type {
        TYPE_ODOM,
        TYPE_LOOP,
    };

    Edge( const g2o::EdgeSE3* edge, Type type );
    Edge( const g2o::EdgeSE3* edge, Type type, GlobalId from_gid, GlobalId to_gid, const GlobalIdGenerator& gid_generator );
    // Edge(const std::string& directory, g2o::HyperGraph* graph);
    virtual ~Edge();

    long                               id() const;
    const Eigen::Isometry3d&           relative_pose() const;
    const Eigen::Matrix<double, 6, 6>& information() const;

public:
    const g2o::EdgeSE3* edge;  // edge instance
    Type                type;
    GlobalId            gid;
    GlobalId            from_gid;
    GlobalId            to_gid;
};


/**
 * @brief EdgeSnapshot for publishing graph
 */
struct EdgeSnapshot {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<EdgeSnapshot>;

    EdgeSnapshot( const Edge::Ptr& edge );
    // EdgeSnapshot(...);

    ~EdgeSnapshot();

public:
    Edge::Type type;
    GlobalId   gid;
    GlobalId   from_gid;
    GlobalId   to_gid;
};

}  // namespace hdl_graph_slam

#endif  // EDGE_HPP
