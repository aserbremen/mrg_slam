// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_TIME_HASH_HPP
#define ROS_TIME_HASH_HPP

#include <boost/functional/hash.hpp>
#include <unordered_map>
// ROS2 migration
#include <builtin_interfaces/msg/time.hpp>

/**
 * @brief Hash calculation for ros::Time
 */
class RosTimeHash {
public:
    size_t operator()( const builtin_interfaces::msg::Time& val ) const
    {
        size_t seed = 0;
        boost::hash_combine( seed, val.sec );
        boost::hash_combine( seed, val.nanosec );
        return seed;
    }
};

#endif  // ROS_TIME_HASHER_HPP
