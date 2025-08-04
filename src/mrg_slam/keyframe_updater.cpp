// SPDX-License-Identifier: BSD-2-Clause

#include <mrg_slam/keyframe_updater.hpp>

namespace mrg_slam {

KeyframeUpdater::KeyframeUpdater( rclcpp::Node::SharedPtr node ) :
    node_( node ), is_first_( true ), accum_distance_( 0.0 ), prev_keypose_( Eigen::Isometry3d::Identity() )
{
}


bool
KeyframeUpdater::update( const Eigen::Isometry3d& pose )
{
    // first frame is always registered to the graph
    if( is_first_ ) {
        is_first_     = false;
        prev_keypose_ = pose;
        return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose_.inverse() * pose;
    double            dx    = delta.translation().norm();
    double            da    = Eigen::AngleAxisd( delta.linear() ).angle();

    // too close to the previous frame
    if( dx < node_->get_parameter( "keyframe_delta_trans" ).as_double()
        && da < node_->get_parameter( "keyframe_delta_angle" ).as_double() ) {
        return false;
    }

    accum_distance_ += dx;
    prev_keypose_ = pose;
    return true;
}


double
KeyframeUpdater::get_accum_distance() const
{
    return accum_distance_;
}


}  // namespace mrg_slam
