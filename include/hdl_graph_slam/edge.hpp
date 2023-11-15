// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_HPP
#define EDGE_HPP

#include <Eigen/Dense>
#include <hdl_graph_slam/keyframe.hpp>
#include <unordered_map>

namespace g2o {
class EdgeSE3;
}  // namespace g2o

namespace hdl_graph_slam {

// Forward declaration for circular dependency
class KeyFrame;

/**
 * @brief KeyFrame (pose node)
 */
struct Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Edge>;

    enum Type {
        TYPE_ANCHOR,
        TYPE_ODOM,
        TYPE_LOOP,
    };

    Edge( const g2o::EdgeSE3* edge, Type type );
    Edge( const g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, std::shared_ptr<const KeyFrame> from_keyframe,
          const boost::uuids::uuid& from_uuid, std::shared_ptr<const KeyFrame> to_keyframe, const boost::uuids::uuid& to_uuid );
    // Edge(const std::string& directory, g2o::HyperGraph* graph);
    virtual ~Edge();

    long                               id() const;
    const Eigen::Isometry3d&           relative_pose() const;
    const Eigen::Matrix<double, 6, 6>& information() const;

    std::string readable_id() const;

public:
    const g2o::EdgeSE3*             edge;  // edge instance
    Type                            type;
    boost::uuids::uuid              uuid;
    std::shared_ptr<const KeyFrame> from_keyframe;
    boost::uuids::uuid              from_uuid;
    std::shared_ptr<const KeyFrame> to_keyframe;
    boost::uuids::uuid              to_uuid;

    // This class should have readable id str member which is created when calling readable_id() for the first time.
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
    Edge::Type         type;
    boost::uuids::uuid uuid;
    boost::uuids::uuid from_uuid;
    boost::uuids::uuid to_uuid;
};

}  // namespace hdl_graph_slam

#endif  // EDGE_HPP
