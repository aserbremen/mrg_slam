// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/global_id.hpp>

#include <algorithm>
#include <stdexcept>


namespace hdl_graph_slam {


GlobalIdGenerator::GlobalIdGenerator(const std::string &own_name, const std::vector<std::string> &robot_names) {
    robot_names_sorted = robot_names;
    std::sort(robot_names_sorted.begin(), robot_names_sorted.end());

    for(size_t i = 0; i < robot_names_sorted.size(); i++) {
        robot_names_mapping[robot_names_sorted[i]] = (RobotId) i;
    }

    own_id = robot_names_mapping[own_name];
    own_id_shifted = ((GlobalId) own_id) << 56;  // 8 bit for robot id, rest of 64 bit for id
}


RobotId GlobalIdGenerator::getRobotId() const {
    return own_id;
}


RobotId GlobalIdGenerator::getRobotId(const std::string &robot_name) const {
    return robot_names_mapping.at(robot_name);
}


RobotId GlobalIdGenerator::getRobotId(const GlobalId &gid) const {
    return gid >> 56;
}


const std::string& GlobalIdGenerator::getRobotName(const RobotId &rid) const {
    return robot_names_sorted[rid];
}


GlobalId GlobalIdGenerator::operator()(int id) const {
    if( id < 0 ) {
        throw std::invalid_argument("Id must be positive");
    }

    GlobalId gid = (GlobalId) id;

    if(gid >= (1 << 56)) {
        throw std::overflow_error("Overflow in global id counter");
    }

    return gid | own_id_shifted;
}

} // namespace hdl_graph_slam