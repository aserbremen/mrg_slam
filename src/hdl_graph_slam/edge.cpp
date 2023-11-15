// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <hdl_graph_slam/edge.hpp>


namespace hdl_graph_slam {

Edge::Edge( const g2o::EdgeSE3* edge, Type type ) :
    edge( edge ), type( type ), uuid( boost::uuids::uuid() ), from_uuid( boost::uuids::uuid() ), to_uuid( boost::uuids::uuid() )
{
}

Edge::Edge( const g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, std::shared_ptr<const KeyFrame> from_keyframe,
            const boost::uuids::uuid& from_uuid, std::shared_ptr<const KeyFrame> to_keyframe, const boost::uuids::uuid& to_uuid ) :
    edge( edge ),
    type( type ),
    uuid( uuid ),
    from_keyframe( from_keyframe ),
    from_uuid( from_uuid ),
    to_keyframe( to_keyframe ),
    to_uuid( to_uuid )
{
}

/*
Edge::Edge(const std::string& directory, g2o::HyperGraph* graph) {
  // TODO: implement
  // load(directory, graph);
}
*/


Edge::~Edge() {}


long
Edge::id() const
{
    return edge->id();
}


const Eigen::Isometry3d&
Edge::relative_pose() const
{
    return edge->measurement();
}


const Eigen::Matrix<double, 6, 6>&
Edge::information() const
{
    return edge->information();
}

std::string
Edge::readable_id() const
{
    std::string readable_id;
    if( type == TYPE_ANCHOR ) {
        readable_id = "anchor." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                      + to_keyframe->robot_name + "-" + std::to_string( to_keyframe->odom_keyframe_counter );
    } else if( type == TYPE_ODOM ) {
        if( to_keyframe != nullptr ) {
            readable_id = "odom." + to_keyframe->robot_name + "-" + std::to_string( to_keyframe->odom_keyframe_counter );
        } else {
            readable_id = "odom.to_keyframe is null";
        }
    } else if( type == TYPE_LOOP ) {
        if( from_keyframe != nullptr && to_keyframe != nullptr ) {
            readable_id = "loop." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                          + to_keyframe->robot_name + "-" + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( from_keyframe == nullptr && to_keyframe != nullptr ) {
            readable_id = "loop.from_keyframe is null ->" + to_keyframe->robot_name + "-"
                          + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( from_keyframe != nullptr && to_keyframe == nullptr ) {
            readable_id = "loop." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter )
                          + "-> to_keyframe is null";
        } else {
            readable_id = "loop from_keyframe is null -> to_keyframe is null";
        }
    } else {
        readable_id = "unknown";
    }

    return readable_id;
}


EdgeSnapshot::EdgeSnapshot( const Edge::Ptr& edge ) :
    type( edge->type ), uuid( edge->uuid ), from_uuid( edge->from_uuid ), to_uuid( edge->to_uuid )
{
}

EdgeSnapshot::~EdgeSnapshot()
{
    // Nothing to do here
}

}  // namespace hdl_graph_slam
