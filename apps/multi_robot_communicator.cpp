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
#include <vamex_slam_msgs/msg/pose_with_name_array.hpp>
#include <vamex_slam_msgs/srv/get_graph_gids.hpp>
#include <vamex_slam_msgs/srv/publish_graph.hpp>

namespace hdl_graph_slam {

class MultiRobotCommunicator : public rclcpp::Node {
public:
    enum class PublishGraphTriggerMode {
        ODOMETRY,  // only considers odometry including drift and no loop closures
        SLAM,      // considers the keyframes that are added to the graph (including loop closures corrections)
    };

    MultiRobotCommunicator() : Node( "multi_robot_communicator" ), communication_ready_( false )
    {
        std::string publish_graph_trigger_mode_str = this->declare_parameter<std::string>( "publish_graph_trigger_mode", "slam" );

        publish_graph_trigger_model_ = trigger_mode_from_string( publish_graph_trigger_mode_str );

        own_name_    = this->declare_parameter<std::string>( "own_name", "atlas" );
        robot_names_ = this->declare_parameter<std::vector<std::string>>( "robot_names", std::vector<std::string>{ "atlas", "bestla" } );
        graph_request_min_accum_dist_ = this->declare_parameter<double>( "graph_request_min_accum_dist", 3.0 );
        graph_request_max_robot_dist_ = this->declare_parameter<double>( "graph_request_max_robot_dist", 10.0 );
        graph_request_min_robot_dist_ = this->declare_parameter<double>( "graph_request_min_robot_dist", 2.0 );
        graph_request_min_time_delay_ = this->declare_parameter<double>( "graph_request_min_time_delay", 5.0 );
        communication_delay_          = this->declare_parameter<int>( "communication_delay", 5 );

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

        // Create the service client to get the graph gids
        std::string get_graph_gids_service_name = "/hdl_graph_slam/get_graph_gids";
        RCLCPP_INFO_STREAM( this->get_logger(), "Creating get graph gids client with service name " << get_graph_gids_service_name );
        get_graph_gids_client_ = this->create_client<vamex_slam_msgs::srv::GetGraphGids>( get_graph_gids_service_name,
                                                                                          rmw_qos_profile_services_default,
                                                                                          reentrant_callback_group );

        // Set the distances from this robot to other robots to a negative value to indicate that they are not known yet
        for( const auto& robot_name : robot_names_ ) {
            if( robot_name == own_name_ ) {
                continue;
            }
            robot_distances_[robot_name]          = std::make_pair( -1.0, -1.0 );
            last_graph_request_times_[robot_name] = builtin_interfaces::msg::Time();
        }

        // Subscribe to the odom broadcast or the others poses topic using the same reentrant callback group as the service clients
        auto subscription_options           = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = reentrant_callback_group;

        if( publish_graph_trigger_model_ == PublishGraphTriggerMode::ODOMETRY ) {
            odom_broadcast_sub_ = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
                "/hdl_graph_slam/odom_broadcast", rclcpp::QoS( 100 ),
                std::bind( &MultiRobotCommunicator::odom_broadcast_callback, this, std::placeholders::_1 ), subscription_options );
            // Initialiaze the tf2 buffer and listener to get odom 2 map transforms
            tf_buffer_   = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );
        } else if( publish_graph_trigger_model_ == PublishGraphTriggerMode::SLAM ) {
            slam_pose_broadcast_sub_ = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
                "/hdl_graph_slam/slam_pose_broadcast", rclcpp::QoS( 100 ),
                std::bind( &MultiRobotCommunicator::slam_pose_broadcast_callback, this, std::placeholders::_1 ), subscription_options );
        }

        // Print all parameters of this node
        print_ros2_parameters( this->get_node_parameters_interface(), this->get_logger() );
    }

    void communication_delay_timer_callback()
    {
        init_timer_->cancel();
        RCLCPP_INFO_STREAM( this->get_logger(), "Communication delay timer callback" );
        communication_ready_ = true;
    }

    void odom_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr odom_msg )
    {
        if( init_timer_ == nullptr ) {
            init_timer_ = this->create_wall_timer( std::chrono::seconds( communication_delay_ ),
                                                   std::bind( &MultiRobotCommunicator::communication_delay_timer_callback, this ) );
            return;
        }

        if( !communication_ready_ ) {
            return;
        }

        // update the pose for this robot
        robot_odom2baselink_transforms_[odom_msg->robot_name] = pose2isometry( odom_msg->pose );
        RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                      "Received odom broadcast for " << odom_msg->robot_name << " with pose\n"
                                                                     << robot_odom2baselink_transforms_[odom_msg->robot_name].matrix() );

        // update the odom2map transforms available on tf2
        update_odom2map_transform( odom_msg->robot_name );

        // update the distances to all robots
        update_distance_odom( *odom_msg );

        // publish the graph if the last graph publish was more than 5 seconds ago
        request_graph_publish( *odom_msg );
    }

    void slam_pose_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg )
    {
        if( init_timer_ == nullptr ) {
            init_timer_ = this->create_wall_timer( std::chrono::seconds( communication_delay_ ),
                                                   std::bind( &MultiRobotCommunicator::communication_delay_timer_callback, this ) );
            return;
        }

        if( !communication_ready_ ) {
            return;
        }

        // update the pose for this robot
        robot_map2baselink_transforms_[slam_pose_msg->robot_name] = pose2isometry( slam_pose_msg->pose );
        RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                      "Received slam pose broadcast for "
                                          << slam_pose_msg->robot_name << " with pose\n"
                                          << robot_map2baselink_transforms_[slam_pose_msg->robot_name].matrix() );

        // update the distances to all robots
        update_distance_slam( *slam_pose_msg );

        // publish the graph if the last graph publish was more than 5 seconds ago
        request_graph_publish( *slam_pose_msg );
    }

    void update_odom2map_transform( const std::string& robot_name )
    {
        // TODO add map frame and odom frame as member variables
        std::string map_frame  = robot_name + "/map";
        std::string odom_frame = robot_name + "/odom";
        if( !tf_buffer_->canTransform( map_frame, odom_frame, rclcpp::Time( 0 ), rclcpp::Duration( 1, 0 ) ) ) {
            RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 500,
                                         "Cannot transform source frame " << odom_frame << " to target frame " << map_frame );
            return;
        }

        geometry_msgs::msg::TransformStamped t_odom2map;
        try {
            t_odom2map = tf_buffer_->lookupTransform( map_frame, odom_frame, rclcpp::Time( 0 ) );
        } catch( tf2::TransformException& ex ) {
            RCLCPP_WARN_STREAM( this->get_logger(), "Could not look up transform: " << ex.what() );
            return;
        }

        robot_odom2map_transforms_[robot_name] = std::make_pair( t_odom2map.transform, tf2::transformToEigen( t_odom2map ) );
        RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                      "Received odom2map transform for " << robot_name << " with transform\n"
                                                                         << tf2::transformToEigen( t_odom2map ).matrix() );
    }

    void update_distance_odom( const vamex_slam_msgs::msg::PoseWithName& pose_msg )
    {
        const auto& robot_name = pose_msg.robot_name;
        if( robot_name == own_name_ ) {
            return;
        }
        auto iter = robot_odom2map_transforms_.find( robot_name );
        if( iter != robot_odom2map_transforms_.end() ) {
            // Calculate the other robot's position in the map frame
            const auto& odom2map      = iter->second.second;
            const auto& odom2baselink = robot_odom2baselink_transforms_[robot_name];
            auto        map2baselink  = odom2map * odom2baselink;
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                          "Calculated map2baselink for " << robot_name << " with transform\n"
                                                                         << map2baselink.matrix() );
            // Calculate this robots position in the map frame
            const auto& odom2map_this_robot      = robot_odom2map_transforms_[own_name_].second;
            const auto& odom2baselink_this_robot = robot_odom2baselink_transforms_[own_name_];
            auto        map2baselink_this_robot  = odom2map_this_robot * odom2baselink_this_robot;
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                          "Calculated map2baselink for " << own_name_ << " with transform\n"
                                                                         << map2baselink_this_robot.matrix() );
            // Calculate the distance between the two robots
            double distance = ( map2baselink.translation() - map2baselink_this_robot.translation() ).norm();
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                          "Calculated distance between " << robot_name << " and " << own_name_ << " as " << distance );

            robot_distances_[robot_name].first = distance;
        }
    }

    void update_distance_slam( const vamex_slam_msgs::msg::PoseWithName& pose_msg )
    {
        const auto& robot_name = pose_msg.robot_name;
        if( robot_name == own_name_ ) {
            return;
        }
        auto iter = robot_map2baselink_transforms_.find( own_name_ );
        if( iter == robot_map2baselink_transforms_.end() ) {
            RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 500,
                                         "Cannot calculate distance to " << robot_name
                                                                         << " because own map2baselink transform is not available" );
            return;
        }

        // Calculate the distance
        const auto& map2baselink            = robot_map2baselink_transforms_[robot_name];
        const auto& map2baselink_this_robot = robot_map2baselink_transforms_[own_name_];

        double distance = ( map2baselink.translation() - map2baselink_this_robot.translation() ).norm();
        RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                      "Calculated distance between " << robot_name << " and " << own_name_ << " as " << distance );

        robot_distances_[robot_name].first = distance;
    }


    void request_graph_publish( const vamex_slam_msgs::msg::PoseWithName& pose_msg )
    {
        const std::string& robot_name = pose_msg.robot_name;
        if( robot_name == own_name_ ) {
            return;
        }

        if( publish_graph_trigger_model_ == PublishGraphTriggerMode::ODOMETRY ) {
            // Check if own transforms are available
            if( robot_odom2map_transforms_.find( own_name_ ) == robot_odom2map_transforms_.end()
                || robot_odom2baselink_transforms_.find( own_name_ ) == robot_odom2baselink_transforms_.end() ) {
                RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 1000,
                                              "Not requesting graph from " << robot_name << " because own transforms are not available" );
                return;
            }
            // Check if other transforms are available
            if( robot_odom2map_transforms_.find( robot_name ) == robot_odom2map_transforms_.end()
                || robot_odom2baselink_transforms_.find( robot_name ) == robot_odom2baselink_transforms_.end() ) {
                RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 1000,
                                              "Not requesting graph from " << robot_name << " because other transforms are not available" );
                return;
            }
        }


        if( ( rclcpp::Time( pose_msg.header.stamp ) - rclcpp::Time( last_graph_request_times_[robot_name] ) ).seconds()
            < graph_request_min_time_delay_ ) {
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 2500,
                                          "Not requesting graph from " << robot_name << " time delay since last graph request is "
                                                                       << ( rclcpp::Time( pose_msg.header.stamp )
                                                                            - rclcpp::Time( last_graph_request_times_[robot_name] ) )
                                                                              .seconds() );
            return;
        }

        // Return if the distance is too large
        if( robot_distances_[robot_name].first > graph_request_max_robot_dist_
            || robot_distances_[robot_name].first < graph_request_min_robot_dist_ ) {
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 1000,
                                          "Not requesting graph from " << robot_name << " because distance is "
                                                                       << robot_distances_[robot_name].first );
            return;
        }

        double& last_accum_dist = robot_distances_[robot_name].second;
        // Check if the other robot has moved enough
        if( fabs( pose_msg.accum_dist - last_accum_dist ) < graph_request_min_accum_dist_ && last_accum_dist >= 0 ) {
            RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 4000,
                                          "Not requesting graph from " << robot_name << " because delta accum_dist is "
                                                                       << fabs( pose_msg.accum_dist - last_accum_dist )
                                                                       << " and last accum distance is " << last_accum_dist );
            return;
        }

        // Get all gids of this robot, check if the service server is available
        while( get_graph_gids_client_->wait_for_service( std::chrono::seconds( 1 ) ) == false ) {
            if( !rclcpp::ok() ) {
                RCLCPP_ERROR_STREAM( this->get_logger(), "Interrupted while waiting for the service "
                                                             << get_graph_gids_client_->get_service_name() << ", returning" );
                return;
            }
            RCLCPP_INFO_STREAM( this->get_logger(),
                                "Service " << get_graph_gids_client_->get_service_name() << " not available, waiting again..." );
        }

        vamex_slam_msgs::srv::GetGraphGids::Request::SharedPtr request_gid =
            std::make_shared<vamex_slam_msgs::srv::GetGraphGids::Request>();
        auto gid_result_future = get_graph_gids_client_->async_send_request( request_gid );

        std::future_status status_gid = gid_result_future.wait_for( std::chrono::seconds( 1 ) );

        if( status_gid == std::future_status::timeout ) {
            RCLCPP_WARN_STREAM( this->get_logger(),
                                "Timeout while waiting for the service " << get_graph_gids_client_->get_service_name() << ", returning" );
            return;
        }
        if( status_gid == std::future_status::ready ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "Sucessfully got own gids" );
        }

        auto own_gids = gid_result_future.get()->gids;

        // check if the service server is available
        while( publish_graph_clients_[robot_name]->wait_for_service( std::chrono::seconds( 1 ) ) == false ) {
            if( !rclcpp::ok() ) {
                RCLCPP_ERROR_STREAM( this->get_logger(), "Interrupted while waiting for the service "
                                                             << publish_graph_clients_[robot_name]->get_service_name() << ", returning" );
                return;
            }
            RCLCPP_INFO_STREAM( this->get_logger(), "Service " << publish_graph_clients_[robot_name]->get_service_name()
                                                               << " not available, waiting again..." );
        }

        vamex_slam_msgs::srv::PublishGraph::Request::SharedPtr request = std::make_shared<vamex_slam_msgs::srv::PublishGraph::Request>();
        request->processed_gids                                        = own_gids;
        RCLCPP_INFO_STREAM( this->get_logger(),
                            "Distance from " << own_name_ << " to " << robot_name << " is " << robot_distances_[robot_name].first << " m" );
        RCLCPP_INFO_STREAM( this->get_logger(), "Requesting graph from " << robot_name );
        auto result_future = publish_graph_clients_[robot_name]->async_send_request( request );

        std::future_status status = result_future.wait_for( std::chrono::seconds( 2 ) );

        if( status == std::future_status::timeout ) {
            RCLCPP_WARN_STREAM( this->get_logger(), "Timeout while waiting for graph from " << robot_name );
            return;
        }
        if( status == std::future_status::ready ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "Successfully requested graph from " << robot_name );
        }

        // Set the last transforms to current transforms
        last_graph_request_times_[robot_name] = pose_msg.header.stamp;
        // Overwrite the last accum distance reference
        last_accum_dist = pose_msg.accum_dist;
    }

private:
    PublishGraphTriggerMode trigger_mode_from_string( std::string& trigger_mode_str )
    {
        std::transform( trigger_mode_str.begin(), trigger_mode_str.end(), trigger_mode_str.begin(),
                        []( unsigned char c ) { return std::tolower( c ); } );
        RCLCPP_INFO_STREAM( this->get_logger(), "Publish graph trigger mode is " << trigger_mode_str );
        if( trigger_mode_str == "odometry" ) {
            return PublishGraphTriggerMode::ODOMETRY;
        } else if( trigger_mode_str == "slam" ) {
            return PublishGraphTriggerMode::SLAM;
        } else {
            RCLCPP_WARN_STREAM( this->get_logger(),
                                "Unknown publish graph trigger mode " << trigger_mode_str << ". Using default value 'odometry'" );
            return PublishGraphTriggerMode::ODOMETRY;
        }
    }

    // TODO std::mutex for odom2map transforms and other values?
    // TODO: possibly check if graph updated at all between robots

    // robot name -> publihsh graph client for that robot
    std::unordered_map<std::string, rclcpp::Client<vamex_slam_msgs::srv::PublishGraph>::SharedPtr> publish_graph_clients_;

    // Get graph gids of own robot to include them in the graph request
    rclcpp::Client<vamex_slam_msgs::srv::GetGraphGids>::SharedPtr get_graph_gids_client_;

    // Subscription to odom broadcast topic, most recent odometry robot poses with drift and without loop closures (odom trigger mode)
    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr odom_broadcast_sub_;

    // tf buffer and listener (odom trigger mode)
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // robot name -> odom2baselink transform for that robot most recently received (odom trigger mode)
    std::unordered_map<std::string, Eigen::Isometry3d> robot_odom2baselink_transforms_;

    // robot name -> odom2map transform for that robot most recently received (odom trigger mode)
    std::unordered_map<std::string, std::pair<geometry_msgs::msg::Transform, Eigen::Isometry3d>> robot_odom2map_transforms_;

    // Subscription to the slam broadcast topic, most recent robot poses with loop closures and corrections (slam trigger mode)
    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr slam_pose_broadcast_sub_;

    // robot name -> map2baselink transform for that robot most recently received (slam trigger mode)
    std::unordered_map<std::string, Eigen::Isometry3d> robot_map2baselink_transforms_;

    // other robot -> (current distance, last accumulated distance)
    std::unordered_map<std::string, std::pair<double, double>> robot_distances_;

    // other robot -> last graph request time
    std::unordered_map<std::string, builtin_interfaces::msg::Time> last_graph_request_times_;

    std::string              own_name_;
    std::vector<std::string> robot_names_;

    rclcpp::TimerBase::SharedPtr init_timer_;

    PublishGraphTriggerMode publish_graph_trigger_model_;

    // Some ROS2 parameters
    double graph_request_min_accum_dist_;
    double graph_request_max_robot_dist_;
    double graph_request_min_robot_dist_;
    double graph_request_min_time_delay_;
    int    communication_delay_;
    bool   communication_ready_;
};

}  // namespace hdl_graph_slam


int
main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    // We need a multi threaded executor in order to process the callbacks and service client calls without deadlocking
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto node = std::make_shared<hdl_graph_slam::MultiRobotCommunicator>();
    executor->add_node( node );
    executor->spin();
    rclcpp::shutdown();

    return 0;
}