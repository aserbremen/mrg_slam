// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_io.hpp>
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
    uuid_str( boost::uuids::to_string( uuid ) ),
    from_keyframe( from_keyframe ),
    from_uuid( from_uuid ),
    from_uuid_str( boost::uuids::to_string( from_uuid ) ),
    to_keyframe( to_keyframe ),
    to_uuid( to_uuid ),
    to_uuid_str( boost::uuids::to_string( to_uuid ) )
{
    // Create the readable id string
    if( type == TYPE_ANCHOR ) {
        readable_id = "anchor." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                      + to_keyframe->robot_name + "-" + std::to_string( to_keyframe->odom_keyframe_counter );
    } else if( type == TYPE_ODOM ) {
        if( to_keyframe != nullptr && from_keyframe != nullptr ) {
            readable_id = "odom." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                          + to_keyframe->robot_name + "-" + std::to_string( to_keyframe->odom_keyframe_counter );
        } else if( to_keyframe == nullptr && from_keyframe != nullptr ) {
            readable_id = "odom." + from_keyframe->robot_name + "-" + std::to_string( from_keyframe->odom_keyframe_counter )
                          + "->to_keyframe is null";
        } else if( to_keyframe != nullptr && from_keyframe == nullptr ) {
            readable_id = "odom.from_keyframe is null->" + to_keyframe->robot_name + "-"
                          + std::to_string( to_keyframe->odom_keyframe_counter );
        } else {
            readable_id = "odom.from_keyframe is null->to_keyframe is null";
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
Edge::save( const std::string& directory )
{
    if( !boost::filesystem::is_directory( directory ) ) {
        boost::filesystem::create_directories( directory );
    }

    // Open the file for appending
    std::ofstream ofs( directory + "/data" );

    ofs << "edge " << readable_id << "\n";
    ofs << "uuid_str " << uuid_str << "\n";
    ofs << "from_uuid_str " << from_uuid_str << "\n";
    ofs << "to_uuid_str " << to_uuid_str << "\n";
    ofs << "g2o_id " << edge->id() << "\n";
    ofs << "relative_pose\n";
    ofs << edge->measurement().matrix() << "\n";
    ofs << "information_matrix\n";
    ofs << edge->information().matrix() << "\n";
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
