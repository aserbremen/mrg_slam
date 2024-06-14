#include <mrg_slam/graph_data_base.hpp>

namespace mrg_slam {

void
GraphDataBase::onInit( rclcpp::Node::SharedPtr _node, std::shared_ptr<GraphSLAM> _graph_slam )
{
    graph_slam = _graph_slam;

    own_name = _node->get_parameter( "own_name" ).as_string();
    // We start the odom_keyframe_counter at 1, because the first keyframe (0) is the anchor keyframe
    odom_keyframe_counter = 1;

    anchor_node     = nullptr;
    anchor_edge_g2o = nullptr;
    anchor_kf       = nullptr;
    anchor_edge_ptr = nullptr;

    prev_robot_keyframe = nullptr;
}

void
GraphDataBase::add_odom_keyframe_to_queue( const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance,
                                           pcl::PointCloud<PointT>::ConstPtr             cloud,
                                           sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
{
    KeyFrame::Ptr kf = std::make_shared<KeyFrame>( stamp, odom, accum_distance, cloud, cloud_msg );
    keyframe_queue.push_back( kf );
    odom_keyframe_counter++;
}

}  // namespace mrg_slam