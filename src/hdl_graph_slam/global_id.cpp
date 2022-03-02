// SPDX-License-Identifier: BSD-2-Clause

#include <ros/time.h>

#include <algorithm>
#include <hdl_graph_slam/global_id.hpp>
#include <stdexcept>


namespace hdl_graph_slam {


GlobalIdGenerator::GlobalIdGenerator( const std::string &own_name, const std::vector<std::string> &robot_names )
{
    robot_names_sorted = robot_names;
    std::sort( robot_names_sorted.begin(), robot_names_sorted.end() );

    for( size_t i = 0; i < robot_names_sorted.size(); i++ ) {
        robot_names_mapping[robot_names_sorted[i]] = (RobotId)i + 1;
    }

    own_id         = robot_names_mapping[own_name];
    own_id_shifted = ( (GlobalId)own_id ) << 56;  // 8bit for robot id, rest of 64bit for id (56bit)

    // get start gid from time to prevent id clashes if one robot should be restartet
    auto time = ros::Time::now();
    // take all 32 bit from sec and use the remaining 24bit (64bit - 32bit - 8bit for robot id) for the upper 24bit of nsec
    start_gid = ( (GlobalId)time.sec ) << 24 | ( (GlobalId)time.nsec ) >> ( 32 - 24 );
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

}  // namespace hdl_graph_slam
