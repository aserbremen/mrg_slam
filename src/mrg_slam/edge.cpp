// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <mrg_slam/edge.hpp>

namespace mrg_slam {

Edge::Edge( const g2o::EdgeSE3* edge, Type type ) :
    edge( edge ), type( type ), uuid( boost::uuids::uuid() ), from_uuid( boost::uuids::uuid() ), to_uuid( boost::uuids::uuid() )
{
}

Edge::Edge( const g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, std::shared_ptr<const KeyFrame> from_keyframe,
            const boost::uuids::uuid& from_uuid, std::shared_ptr<const KeyFrame> to_keyframe, const boost::uuids::uuid& to_uuid ) :
    edge( edge ),
    type( type ),
    uuid( uuid ),
    uuid_str( boost::uuids::to_string( uuid ) ),
    from_keyframe( from_keyframe ),
    from_uuid( from_uuid ),
    from_uuid_str( boost::uuids::to_string( from_uuid ) ),
    to_keyframe( to_keyframe ),
    to_uuid( to_uuid ),
    to_uuid_str( boost::uuids::to_string( to_uuid ) )
{
    std::string robot_name = from_keyframe->robot_name.empty() ? "\"\"" : from_keyframe->robot_name;
    // Create the readable id string
    if( type == TYPE_ANCHOR ) {
        readable_id = "anchor." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->" + to_keyframe->robot_name
                      + "-" + std::to_string( to_keyframe->odom_keyframe_counter );

        return;
    }

    if( type == TYPE_ODOM ) {
        if( to_keyframe != nullptr && from_keyframe != nullptr ) {
            readable_id = "odom." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                          + to_keyframe->robot_name + "." + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( to_keyframe == nullptr && from_keyframe != nullptr ) {
            readable_id = "odom." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->to_keyframe is null";
        } else if( to_keyframe != nullptr && from_keyframe == nullptr ) {
            readable_id = "odom.from_keyframe is null->" + to_keyframe->robot_name + "."
                          + std::to_string( to_keyframe->odom_keyframe_counter );
        } else {
            readable_id = "odom.from_keyframe is null->to_keyframe is null";
        }

        return;
    }

    if( type == TYPE_LOOP ) {
        if( from_keyframe != nullptr && to_keyframe != nullptr ) {
            readable_id = "loop." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                          + to_keyframe->robot_name + "." + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( from_keyframe == nullptr && to_keyframe != nullptr ) {
            readable_id = "loop.from_keyframe is null ->" + to_keyframe->robot_name + "."
                          + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( from_keyframe != nullptr && to_keyframe == nullptr ) {
            readable_id = "loop." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "-> to_keyframe is null";
        } else {
            readable_id = "loop from_keyframe is null -> to_keyframe is null";
        }

        return;
    }

    std::runtime_error( "Unknown edge type in Edge constructor" );
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

void
Edge::save( const std::string& result_path )
{
    std::ofstream ofs( result_path + ".txt" );

    ofs << "edge " << readable_id << "\n";
    std::string type_str;
    if( type == TYPE_ANCHOR ) {
        type_str = "anchor";
    } else if( type == TYPE_ODOM ) {
        type_str = "odom";
    } else if( type == TYPE_LOOP ) {
        type_str = "loop";
    } else {
        type_str = "unknown";
    }
    ofs << "type " << type_str << "\n";
    ofs << "from " << from_keyframe->readable_id << "\n";
    ofs << "to " << to_keyframe->readable_id << "\n";
    ofs << "g2o_id " << edge->id() << "\n";
    ofs << "relative_pose\n";
    ofs << edge->measurement().matrix() << "\n";
    ofs << "information_matrix\n";
    ofs << edge->information().matrix() << "\n";
    ofs << "uuid_str " << uuid_str << "\n";
    ofs << "from_uuid_str " << from_uuid_str << "\n";
    ofs << "to_uuid_str " << to_uuid_str << "\n";
}


EdgeSnapshot::EdgeSnapshot( const Edge::Ptr& edge ) :
    type( edge->type ), uuid( edge->uuid ), from_uuid( edge->from_uuid ), to_uuid( edge->to_uuid )
{
}

EdgeSnapshot::~EdgeSnapshot()
{
    // Nothing to do here
}

}  // namespace mrg_slam
