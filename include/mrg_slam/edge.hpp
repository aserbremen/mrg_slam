// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_HPP
#define EDGE_HPP

#include <Eigen/Dense>
#include <boost/uuid/uuid.hpp>
#include <mrg_slam/keyframe.hpp>
#include <unordered_map>

namespace g2o {
class EdgeSE3;
}  // namespace g2o

namespace mrg_slam {

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

    Edge( g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, const std::string& uuid_str,
          std::shared_ptr<const KeyFrame> from_keyframe, std::shared_ptr<const KeyFrame> to_keyframe );
    Edge( const std::string& edge_path, const boost::uuids::uuid& _uuid, const std::string& _uuid_str, const boost::uuids::uuid& _from_uuid,
          const std::string& _from_uuid_str, const boost::uuids::uuid& _to_uuid, const std::string& _to_uuid_str );
    virtual ~Edge();

    long                               id() const;
    const Eigen::Isometry3d&           relative_pose() const;
    const Eigen::Matrix<double, 6, 6>& information() const;

    void save( const std::string& result_path );
    void load( const std::string& edge_path, const boost::uuids::uuid& _uuid, const std::string& _uuid_str,
               const boost::uuids::uuid& _from_uuid, const std::string& _from_uuid_str, const boost::uuids::uuid& _to_uuid,
               const std::string& _to_uuid_str );

public:
    g2o::EdgeSE3*                   edge;           // edge instance
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

    Eigen::Isometry3d           relative_pose_loaded;  // relative pose from loaded edge
    Eigen::Matrix<double, 6, 6> information_loaded;    // information matrix from loaded edge

private:
    void make_readable_id();

    Type        type_from_string( const std::string& type_str );
    std::string type_to_string( Type type );
};

/**
 * @brief EdgeSnapshot for publishing graph
 */
struct EdgeSnapshot {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<EdgeSnapshot>;

    EdgeSnapshot( const Edge::Ptr& edge );

    ~EdgeSnapshot();

public:
    // TODO find out whether to use uuid or uuid_str
    Edge::Type         type;
    boost::uuids::uuid uuid;
    boost::uuids::uuid from_uuid;
    boost::uuids::uuid to_uuid;
};

}  // namespace mrg_slam

#endif  // EDGE_HPP
