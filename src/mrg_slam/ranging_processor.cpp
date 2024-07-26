#include <mrg_slam/ranging_processor.hpp>

namespace mrg_slam {

RangingProcessor::RangingProcessor() {}

void
RangingProcessor::onInit( rclcpp::Node::SharedPtr _node )
{
    node = _node;

    own_name = node->get_parameter( "own_name" ).as_string();

    ranging_sub_topic = node->declare_parameter( "ranging_sub_topic", "/ranging_measurement" );
    ranging_names     = node->declare_parameter( "ranging_names", std::vector<std::string>() );

    std::string prefix = own_name.empty() ? "" : "/" + own_name;
    ranging_sub_topic  = prefix + ranging_sub_topic;

    ranging_sub = node->create_subscription<ros2_radio_ranging_interfaces::msg::Range>( ranging_sub_topic, rclcpp::QoS( 10 ),
                                                                                        std::bind( &RangingProcessor::ranging_callback,
                                                                                                   this, std::placeholders::_1 ) );
}

void
RangingProcessor::ranging_callback( ros2_radio_ranging_interfaces::msg::Range::ConstSharedPtr ranging_msg )
{
    std::lock_guard<std::mutex> lock( ranging_queue_mutex );
    ranging_queue.push_back( ranging_msg );
}

bool
RangingProcessor::flush( std::shared_ptr<GraphSLAM> &graph_slam, const std::vector<KeyFrame::Ptr> &keyframes )
{
    std::lock_guard<std::mutex> lock( ranging_queue_mutex );
    if( ranging_queue.empty() || keyframes.empty() ) {
        return false;
    }

    for( const auto &ranging_msg : ranging_queue ) {
        for( const auto &keyframe : keyframes ) {
            if( std::abs( ( rclcpp::Time( ranging_msg->header.stamp ) - rclcpp::Time( keyframe->stamp ) ).seconds() ) < 0.1 ) {
                RCLCPP_INFO_STREAM( node->get_logger(), "Ranging message from " << ranging_msg->self_name << " to "
                                                                                << ranging_msg->neighbor_name << " close to "
                                                                                << keyframe->readable_id );
            }
        }
    }

    return false;  // TODO
}


}  // namespace mrg_slam
