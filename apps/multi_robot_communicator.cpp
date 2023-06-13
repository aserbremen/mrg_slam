#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <builtin_interfaces/msg/time.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hdl_graph_slam/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_map>
#include <utility>
#include <vamex_slam_msgs/msg/pose_with_name.hpp>
#include <vamex_slam_msgs/srv/publish_graph.hpp>

namespace hdl_graph_slam {

// Custom hash function for std::pair<std::string, std::string>
struct PairHash {
    template<typename T1, typename T2>
    std::size_t operator()( const std::pair<T1, T2>& pair ) const
    {
        auto hash1 = std::hash<T1>{}( pair.first );
        auto hash2 = std::hash<T2>{}( pair.second );
        return hash1 ^ hash2;
    }
};


class MultiRobotCommunicator : public rclcpp::Node {
public:
    MultiRobotCommunicator() : Node( "multi_robot_communicator" )
    {
        robot_names_ = this->declare_parameter<std::vector<std::string>>( "robot_names", std::vector<std::string>{ "atlas", "bestla" } );
        own_name_    = this->declare_parameter<std::string>( "own_name", "atlas" );

        std::string service_name = "/hdl_graph_slam/publish_graph";
        if( robot_names_.empty() ) {
            RCLCPP_WARN( this->get_logger(), "No robot names specified." );
            RCLCPP_INFO_STREAM( this->get_logger(), "Creating publish graph client with default service name " << service_name );
        }

        // Use a reentrant callback group for the service clients and any callback in which the service client is used
        // TODO test if multiple reentrant callback groups are needed for multiple publish graph clients
        rclcpp::CallbackGroup::SharedPtr reentrant_callback_group = this->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
        for( const auto& robot_name : robot_names_ ) {
            if( robot_name == own_name_ ) {
                continue;
            }
            service_name = "/" + robot_name + "/hdl_graph_slam/publish_graph";
            RCLCPP_INFO_STREAM( this->get_logger(),
                                "Creating publish graph client for robot " << robot_name << " with service name " << service_name );
            publish_graph_clients_[robot_name] = this->create_client<vamex_slam_msgs::srv::PublishGraph>( service_name,
                                                                                                          rmw_qos_profile_services_default,
                                                                                                          reentrant_callback_group );
        }

        // Set the distances from this robot to other robots to a negative value to indicate that they are not known yet
        for( const auto& robot_name : robot_names_ ) {
            if( robot_name == own_name_ ) {
                continue;
            }
            robot_distances_[robot_name] = -1.0;
        }

        // Subscribe to the odom broadcast topic
        auto subscription_options           = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = reentrant_callback_group;
        odom_broadcast_sub_                 = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
            "/hdl_graph_slam/odom_broadcast", rclcpp::QoS( 20 ),
            std::bind( &MultiRobotCommunicator::odom_broadcast_callback, this, std::placeholders::_1 ), subscription_options );

        // Initialiaze the tf2 buffer and listener to get odom 2 map transforms
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

        // Print all parameters of this node
        std::vector<rclcpp::Parameter> params_vec = this->get_parameters( this->list_parameters( std::vector<std::string>{}, 0 ).names );
        print_ros2_parameters( params_vec, this->get_logger() );
    }

    void odom_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr odom_msg )
    {
        RCLCPP_INFO_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 2000,
                                     "Received odom broadcast from " << odom_msg->robot_name );

        // update the pose for this robot
        robot_odom2baselink_transforms_[odom_msg->robot_name] = pose2isometry( odom_msg->pose );
        RCLCPP_INFO_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                     "Received odom broadcast for " << odom_msg->robot_name << " with pose\n"
                                                                    << pose2isometry( odom_msg->pose ).matrix() );

        // update the odom2map transforms available on tf2
        update_odom2map_transforms();

        // update the distances to all robots
        update_distances();

        if( odom_msg->robot_name == own_name_ ) {
            return;
        }

        if( last_publish_graph_time_.sec == 0 && last_publish_graph_time_.nanosec == 0 ) {
            last_publish_graph_time_ = odom_msg->header.stamp;
            return;
        }

        // publish the graph if the last graph publish was more than 5 seconds ago
        if( ( rclcpp::Time( odom_msg->header.stamp ) - rclcpp::Time( last_publish_graph_time_ ) ).seconds() > 5 ) {
            last_publish_graph_time_ = rclcpp::Time( odom_msg->header.stamp );
            // Request graph from all robots
            for( const auto& robot_name : robot_names_ ) {
                if( robot_name == own_name_ ) {
                    continue;
                }

                // check if the service server is available
                while( publish_graph_clients_[robot_name]->wait_for_service( std::chrono::seconds( 1 ) ) == false ) {
                    if( !rclcpp::ok() ) {
                        RCLCPP_ERROR_STREAM( this->get_logger(), "Interrupted while waiting for the service "
                                                                     << publish_graph_clients_[robot_name]->get_service_name()
                                                                     << " Exiting" );
                        return;
                    }
                    RCLCPP_INFO_STREAM( this->get_logger(), "Service " << publish_graph_clients_[robot_name]->get_service_name()
                                                                       << " not available, waiting again..." );
                }

                vamex_slam_msgs::srv::PublishGraph::Request::SharedPtr request =
                    std::make_shared<vamex_slam_msgs::srv::PublishGraph::Request>();
                RCLCPP_INFO_STREAM( this->get_logger(), "Requesting graph from " << robot_name );
                auto result_future = publish_graph_clients_[robot_name]->async_send_request( request );

                std::future_status status = result_future.wait_for( std::chrono::seconds( 2 ) );

                if( status == std::future_status::ready ) {
                    RCLCPP_INFO_STREAM( this->get_logger(), "Successfully requested graph from " << robot_name );
                }
                if( status == std::future_status::timeout ) {
                    RCLCPP_WARN_STREAM( this->get_logger(), "Timeout while waiting for graph from " << robot_name );
                    last_publish_graph_time_ = rclcpp::Time( odom_msg->header.stamp );
                    return;
                }
                auto result = result_future.get();
                // Nothing else to do here
            }
        }
    }

    void update_odom2map_transforms()
    {
        // TODO add map frame and odom frame as member variables
        for( const auto& robot_name : robot_names_ ) {
            std::string map_frame  = robot_name + "/map";
            std::string odom_frame = robot_name + "/odom";
            if( !tf_buffer_->canTransform( map_frame, odom_frame, rclcpp::Time( 0 ), rclcpp::Duration( 1, 0 ) ) ) {
                RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 500,
                                             "Cannot transform source frame " << odom_frame << " to target frame " << map_frame );
                continue;
            }

            geometry_msgs::msg::TransformStamped t_odom2map;
            try {
                t_odom2map = tf_buffer_->lookupTransform( map_frame, odom_frame, rclcpp::Time( 0 ) );
            } catch( tf2::TransformException& ex ) {
                RCLCPP_WARN( this->get_logger(), "Could not look up transform: %s", ex.what() );
                continue;
            }

            robot_odom2map_transforms_[robot_name] = std::make_pair( t_odom2map.transform, tf2::transformToEigen( t_odom2map ) );
            RCLCPP_INFO_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                         "Received odom2map transform for " << robot_name << " with transform\n"
                                                                            << tf2::transformToEigen( t_odom2map ).matrix() );
        }
    }

    void update_distances()
    {
        for( const auto& robot_name : robot_names_ ) {
            if( robot_name == own_name_ ) {
                continue;
            }
            // Calculate the other robot's position in the shared map frame
        }
    }

private:
    // robot name -> publihsh graph client for that robot
    std::unordered_map<std::string, rclcpp::Client<vamex_slam_msgs::srv::PublishGraph>::SharedPtr> publish_graph_clients_;

    // robot name -> odom2baselink transform for that robot most recently received
    std::unordered_map<std::string, Eigen::Isometry3d> robot_odom2baselink_transforms_;

    // robot name -> odom2map transform for that robot most recently received
    std::unordered_map<std::string, std::pair<geometry_msgs::msg::Transform, Eigen::Isometry3d>> robot_odom2map_transforms_;

    // other robot -> distance between this robot and other robot
    std::unordered_map<std::string, double> robot_distances_;

    // other robot -> last transforms the graphs were exchanged (own robot, other robot)
    std::unordered_map<std::string, std::string, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> last_publish_graph_poses_;


    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr odom_broadcast_sub_;

    std::string              own_name_;
    std::vector<std::string> robot_names_;

    builtin_interfaces::msg::Time last_publish_graph_time_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hdl_graph_slam


int
main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    // TODO decide if multi_threaded_executor is needed
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto node = std::make_shared<hdl_graph_slam::MultiRobotCommunicator>();
    executor->add_node( node );
    executor->spin();
    rclcpp::shutdown();

    return 0;
}