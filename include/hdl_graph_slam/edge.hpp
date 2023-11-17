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

    void save( const std::string& result_path );

public:
    const g2o::EdgeSE3*             edge;           // edge instance
    Type                            type;           // edge type
    boost::uuids::uuid              uuid;           // unique id for this edge
    std::string                     uuid_str;       // unique id for this edge as a string for graph exchange
    std::shared_ptr<const KeyFrame> from_keyframe;  // from keyframe pointer
    boost::uuids::uuid              from_uuid;      // from keyframe uuid
    std::string                     from_uuid_str;  // from keyframe uuid as a string for graph exchange
    std::shared_ptr<const KeyFrame> to_keyframe;    // to keyframe pointer
    boost::uuids::uuid              to_uuid;        // to keyframe uuid
    std::string                     to_uuid_str;    // to keyframe uuid as a string for graph exchange
    std::string                     readable_id;    // readable id for visualizing and debugging
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
    // TODO find out whether to use uuid or uuid_str
    Edge::Type         type;
    boost::uuids::uuid uuid;
    boost::uuids::uuid from_uuid;
    boost::uuids::uuid to_uuid;
};

}  // namespace hdl_graph_slam

#endif  // EDGE_HPP
