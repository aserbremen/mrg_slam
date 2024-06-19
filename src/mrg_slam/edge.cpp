// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <mrg_slam/edge.hpp>

namespace mrg_slam {

Edge::Edge( const g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, const std::string& uuid_str,
            std::shared_ptr<const KeyFrame> from_keyframe, std::shared_ptr<const KeyFrame> to_keyframe ) :
    edge( edge ), type( type ), uuid( uuid ), uuid_str( uuid_str ), from_keyframe( from_keyframe ), to_keyframe( to_keyframe )
{
    readable_id = make_readable_id();
}

Edge::Edge( const std::string& edge_path ) { load( edge_path ); }

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
    std::string type_str = type_to_string( type );
    ofs << "type " << type_str << "\n";
    ofs << "from " << from_keyframe->readable_id << "\n";
    ofs << "to " << to_keyframe->readable_id << "\n";
    ofs << "g2o_id " << edge->id() << "\n";
    ofs << "relative_pose\n";
    ofs << edge->measurement().matrix() << "\n";
    ofs << "information_matrix\n";
    ofs << edge->information().matrix() << "\n";
    ofs << "uuid_str " << uuid_str << "\n";
    ofs << "from_uuid_str " << from_keyframe->uuid_str << "\n";
    ofs << "to_uuid_str " << to_keyframe->uuid_str << "\n";
}

void
Edge::load( const std::string& edge_path )
{
    auto logger = rclcpp::get_logger( "load_edge" );

    std::ifstream ifs( edge_path );

    std::string line;
    while( std::getline( ifs, line ) ) {
        std::stringstream iss( line );
        std::string       key;
        iss >> key;
        // we skip keys <from> and <to> which are given by the respective keyframe pointer
        if( key == "edge" ) {
            iss >> readable_id;
        } else if( key == "type" ) {
            std::string type_str;
            iss >> type_str;
            type = type_from_string( type_str );
        } else if( key == "uuid_str" ) {
            iss >> uuid_str;
        }
        // dont load from and to keyframes uuid_str here, because they are loaded from the keyframes
        // TODO take care of g2o_id, relative_pose, information_matrix... which are graph related
    }

    // print the loaded edge information
    RCLCPP_INFO( logger, "Loaded edge %s", readable_id.c_str() );
    RCLCPP_INFO( logger, "type %s", type_to_string( type ).c_str() );
    RCLCPP_INFO( logger, "uuid_str %s", uuid_str.c_str() );
}

std::string
Edge::make_readable_id()
{
    std::string robot_name = from_keyframe->robot_name.empty() ? "\"\"" : from_keyframe->robot_name;
    std::string readable_id;
    if( type == TYPE_ANCHOR ) {
        readable_id = "anchor." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->" + to_keyframe->robot_name
                      + "-" + std::to_string( to_keyframe->odom_keyframe_counter );

        return readable_id;
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

        return readable_id;
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

        return readable_id;
    }

    std::runtime_error( "Unknown edge type in Edge constructor" );
}

Edge::Type
Edge::type_from_string( const std::string& type_str )
{
    if( type_str == "anchor" ) {
        return TYPE_ANCHOR;
    } else if( type_str == "odom" ) {
        return TYPE_ODOM;
    } else if( type_str == "loop" ) {
        return TYPE_LOOP;
    } else {
        std::runtime_error( "Unknown edge type in Edge::type_from_string" );
    }
}

std::string
Edge::type_to_string( Type type )
{
    if( type == TYPE_ANCHOR ) {
        return "anchor";
    } else if( type == TYPE_ODOM ) {
        return "odom";
    } else if( type == TYPE_LOOP ) {
        return "loop";
    } else {
        std::runtime_error( "Unknown edge type in Edge::string_from_type" );
    }
}


EdgeSnapshot::EdgeSnapshot( const Edge::Ptr& edge ) :
    type( edge->type ), uuid( edge->uuid ), from_uuid( edge->from_keyframe->uuid ), to_uuid( edge->to_keyframe->uuid )
{
}

EdgeSnapshot::~EdgeSnapshot()
{
    // Nothing to do here
}

}  // namespace mrg_slam
