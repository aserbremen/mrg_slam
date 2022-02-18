// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_TIME_HASH_HPP
#define ROS_TIME_HASH_HPP

#include <ros/time.h>

#include <boost/functional/hash.hpp>
#include <unordered_map>

/**
 * @brief Hash calculation for ros::Time
 */
class RosTimeHash {
public:
    size_t operator()( const ros::Time& val ) const
    {
        size_t seed = 0;
        boost::hash_combine( seed, val.sec );
        boost::hash_combine( seed, val.nsec );
        return seed;
    }
};

#endif  // ROS_TIME_HASHER_HPP
