// SPDX-License-Identifier: BSD-2-Clause

#include <mrg_slam/keyframe_updater.hpp>

namespace mrg_slam {

KeyframeUpdater::KeyframeUpdater( rclcpp::Node::SharedPtr _node ) : is_first( true ), prev_keypose( Eigen::Isometry3d::Identity() )
{
    keyframe_delta_trans = _node->get_parameter( "keyframe_delta_trans" ).as_double();
    keyframe_delta_angle = _node->get_parameter( "keyframe_delta_angle" ).as_double();

    accum_distance = 0.0;
}


bool
KeyframeUpdater::update( const Eigen::Isometry3d& pose )
{
    // first frame is always registered to the graph
    if( is_first ) {
        is_first     = false;
        prev_keypose = pose;
        return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double            dx    = delta.translation().norm();
    double            da    = Eigen::AngleAxisd( delta.linear() ).angle();

    // too close to the previous frame
    if( dx < keyframe_delta_trans && da < keyframe_delta_angle ) {
        return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    return true;
}


double
KeyframeUpdater::get_accum_distance() const
{
    return accum_distance;
}


}  // namespace mrg_slam
