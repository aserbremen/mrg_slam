// SPDX-License-Identifier: BSD-2-Clause

#include <rclcpp/time.hpp>
// #include <ros/time.h>

#include <algorithm>
#include <hdl_graph_slam/global_id.hpp>
#include <stdexcept>


namespace hdl_graph_slam {


GlobalIdGenerator::GlobalIdGenerator( rclcpp::Node::SharedPtr _node, const std::string &own_name,
                                      const std::vector<std::string> &robot_names )
{
    robot_names_sorted = robot_names;
    std::sort( robot_names_sorted.begin(), robot_names_sorted.end() );

    for( size_t i = 0; i < robot_names_sorted.size(); i++ ) {
        robot_names_mapping[robot_names_sorted[i]] = (RobotId)i + 1;
    }

    own_id         = robot_names_mapping[own_name];
    own_id_shifted = ( (GlobalId)own_id ) << 56;  // 8bit for robot id, rest of 64bit for id (56bit)

    // get start gid from time to prevent id clashes if one robot should be restartet
    // convert to builtin_interfaces::msg::Time to get acces to sec and nanosec members
    // auto time = ros::Time::now();  // ROS1
    auto time = _node->now().operator builtin_interfaces::msg::Time();

    // take all 32 bit from sec and use the remaining 24bit (64bit - 32bit - 8bit for robot id) for the upper 24bit of nsec
    // start_gid = ( (GlobalId)time.sec ) << 24 | ( (GlobalId)time.nsec ) >> ( 32 - 24 );
    // TODO verify with Joachim if the gid is still correctly generated in ROS2
    // ROS1 ros::Time::Time( uint32_t _sec, uint32_t _nsec), ROS2 builtin_interfaces::msg::Time::Time( int32_t sec, uint32_t nanosec)
    start_gid = ( (GlobalId)time.sec ) << 24 | ( (GlobalId)time.nanosec ) >> ( 32 - 24 );

    robot_names_start_gid_mapping[own_name] = start_gid;
}


RobotId
GlobalIdGenerator::getRobotId() const
{
    return own_id;
}


RobotId
GlobalIdGenerator::getRobotId( const std::string &robot_name ) const
{
    return robot_names_mapping.at( robot_name );
}


RobotId
GlobalIdGenerator::getRobotId( const GlobalId &gid ) const
{
    return gid >> 56;
}


const std::string &
GlobalIdGenerator::getRobotName( const RobotId &rid ) const
{
    return robot_names_sorted[rid - 1];
}


GlobalId
GlobalIdGenerator::operator()( int id ) const
{
    if( id < 0 ) {
        throw std::invalid_argument( "Id must be positive" );
    }

    GlobalId gid = (GlobalId)id + start_gid;

    if( gid >= ( ( (uint64_t)1 ) << 56 ) ) {
        throw std::overflow_error( "Overflow in global id counter" );
    }

    return gid | own_id_shifted;
}

std::string
GlobalIdGenerator::getHumanReadableId( GlobalId gid, bool with_start_gid ) const
{
    if( gid == 0 ) {
        return "fixed_node-GID(0)";
    }
    RobotId     rid        = getRobotId( gid );
    std::string robot_name = getRobotName( rid );
    int         id         = gid - robot_names_start_gid_mapping.at( robot_name );
    if( with_start_gid ) {
        return robot_name + "#" + std::to_string( start_gid ) + "-" + std::to_string( id );
    }
    return robot_name + "-" + std::to_string( id );
}

GlobalId
GlobalIdGenerator::getStartGid( const std::string &robot_name ) const
{
    return robot_names_start_gid_mapping.at( robot_name );
}

int
GlobalIdGenerator::getIdWithoutStartGid( GlobalId gid ) const
{
    if( gid == 0 ) {
        // GlobalId 0 is reserved for fixed nodes which is 1 in our case
        return 1;
    }
    RobotId     rid        = getRobotId( gid );
    std::string robot_name = getRobotName( rid );
    int         id         = gid - robot_names_start_gid_mapping.at( robot_name );
    return id;
}

// TODO Change this function so it can handle restarts of robots with different start_gids
void
GlobalIdGenerator::addStartGid( const std::string &robot_name, GlobalId other_start_gid )
{
    robot_names_start_gid_mapping.insert( std::pair<std::string, GlobalId>( robot_name, other_start_gid ) );
    std::cout << "Addded " << robot_name << "#" << other_start_gid << " to start_gids" << std::endl;
}


}  // namespace hdl_graph_slam
