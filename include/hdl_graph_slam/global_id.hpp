// SPDX-License-Identifier: BSD-2-Clause

#ifndef GLOBAL_ID_HPP
#define GLOBAL_ID_HPP

#include <inttypes.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace hdl_graph_slam {

typedef uint64_t GlobalId;
typedef uint8_t  RobotId;

class GlobalIdGenerator {
public:
    GlobalIdGenerator( const std::string &own_name, const std::vector<std::string> &robot_names );

    RobotId getRobotId() const;
    RobotId getRobotId( const std::string &robot_name ) const;
    RobotId getRobotId( const GlobalId &gid ) const;

    const std::string &getRobotName( const RobotId &rid ) const;

    GlobalId operator()( int id ) const;

protected:
    std::vector<std::string>                 robot_names_sorted;
    std::unordered_map<std::string, RobotId> robot_names_mapping;

    RobotId  own_id;
    GlobalId own_id_shifted;
};


}  // namespace hdl_graph_slam

#endif  // GLOBAL_ID_HPP