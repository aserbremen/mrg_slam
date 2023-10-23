// SPDX-License-Identifier: BSD-2-Clause

#ifndef GLOBAL_ID_HPP
#define GLOBAL_ID_HPP

#include <inttypes.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace hdl_graph_slam {

typedef uint64_t GlobalId;
typedef uint8_t  RobotId;

// TODO: refactor this class to be more generic and easier to use

class GlobalIdGenerator {
public:
    // ROS2 migration, also pass the node ptr to get ROS2 time
    GlobalIdGenerator( rclcpp::Node::SharedPtr _node, const std::string &own_name, const std::vector<std::string> &robot_names );

    RobotId getRobotId() const;
    RobotId getRobotId( const std::string &robot_name ) const;
    RobotId getRobotId( const GlobalId &gid ) const;

    const std::string &getRobotName( const RobotId &rid ) const;
    std::string        getHumanReadableId( GlobalId gid, bool with_start_gid = false ) const;
    GlobalId           getStartGid( const std::string &robot_name ) const;
    void               addStartGid( const std::string &robot_name, GlobalId other_start_gid );
    // Returns the id without the start_gid, e.g. 0, 2, 3, ...
    int getIdWithoutStartGid( GlobalId gid ) const;

    GlobalId operator()( int id ) const;

protected:
    std::vector<std::string>                  robot_names_sorted;
    std::unordered_map<std::string, RobotId>  robot_names_mapping;
    std::unordered_map<std::string, GlobalId> robot_names_start_gid_mapping;

    RobotId  own_id;
    GlobalId own_id_shifted;
    GlobalId start_gid;
};


}  // namespace hdl_graph_slam

#endif  // GLOBAL_ID_HPP