// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>

#include <boost/filesystem.hpp>
#include <mrg_slam/edge.hpp>

namespace mrg_slam {

Edge::Edge( const g2o::EdgeSE3* edge, Type type, const boost::uuids::uuid& uuid, const std::string& uuid_str,
            std::shared_ptr<const KeyFrame> from_keyframe, std::shared_ptr<const KeyFrame> to_keyframe ) :
    edge( edge ),
    type( type ),
    uuid( uuid ),
    uuid_str( uuid_str ),
    from_keyframe( from_keyframe ),
    from_uuid( from_keyframe->uuid ),
    from_uuid_str( from_keyframe->uuid_str ),
    to_keyframe( to_keyframe ),
    to_uuid( to_keyframe->uuid ),
    to_uuid_str( to_keyframe->uuid_str )
{
    readable_id = make_readable_id();
}

Edge::Edge( const std::string& edge_path, const boost::uuids::uuid& _uuid, const std::string& _uuid_str,
            const boost::uuids::uuid& _from_uuid, const std::string& _from_uuid_str, const boost::uuids::uuid& _to_uuid,
            const std::string& _to_uuid_str )
{
    load( edge_path, _uuid, _uuid_str, _from_uuid, _from_uuid_str, _to_uuid, _to_uuid_str );
}

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
    ofs << "from_uuid_str " << from_uuid_str << "\n";
    ofs << "to_uuid_str " << to_uuid_str << "\n";
}

void
Edge::load( const std::string& edge_path, const boost::uuids::uuid& _uuid, const std::string& _uuid_str,
            const boost::uuids::uuid& _from_uuid, const std::string& _from_uuid_str, const boost::uuids::uuid& _to_uuid,
            const std::string& _to_uuid_str )
{
    auto logger = rclcpp::get_logger( "load_edge" );

    uuid          = _uuid;
    uuid_str      = _uuid_str;
    from_uuid     = _from_uuid;
    from_uuid_str = _from_uuid_str;
    to_uuid       = _to_uuid;
    to_uuid_str   = _to_uuid_str;

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
        } else if( key == "relative_pose" ) {
            Eigen::Matrix4d& matrix = relative_pose_loaded.matrix();
            for( int i = 0; i < 4; i++ ) {
                for( int j = 0; j < 4; j++ ) {
                    iss >> matrix( i, j );
                }
            }
        } else if( key == "information_matrix" ) {
            Eigen::Matrix<double, 6, 6>& matrix = information_loaded;
            for( int i = 0; i < 6; i++ ) {
                for( int j = 0; j < 6; j++ ) {
                    iss >> matrix( i, j );
                }
            }
        }
    }

    // print the loaded edge information
    RCLCPP_INFO_STREAM( logger, "Loaded edge " << readable_id );
    RCLCPP_INFO_STREAM( logger, "type " << type_to_string( type ) );
    RCLCPP_INFO_STREAM( logger, "uuid_str " << uuid_str );
    RCLCPP_INFO_STREAM( logger, "from_uuid_str " << from_uuid_str );
    RCLCPP_INFO_STREAM( logger, "to_uuid_str " << to_uuid_str );
    RCLCPP_INFO_STREAM( logger, "relative_pose\n" << relative_pose_loaded.matrix() );
    RCLCPP_INFO_STREAM( logger, "information_matrix\n" << information_loaded );
}

std::string
Edge::make_readable_id()
{
    std::string robot_name = from_keyframe->robot_name.empty() ? "\"\"" : from_keyframe->robot_name;
    std::string readable_id;
    if( type == TYPE_ANCHOR ) {
        readable_id = "anchor." + robot_name + "." + std::to_string( from_keyframe->odom_keyframe_counter ) + "->"
                      + std::to_string( to_keyframe->odom_keyframe_counter );

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
