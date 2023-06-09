#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    MultiRobotCommunicator() : Node( "multi_robot_service_client" )
    {
        robot_names_ = this->declare_parameter<std::vector<std::string>>( "robot_names", std::vector<std::string>{ "atlas", "bestla" } );

        std::string service_name = "/publish_graph";
        if( robot_names_.empty() ) {
            RCLCPP_WARN( this->get_logger(), "No robot names specified." );
            RCLCPP_INFO( this->get_logger(), "Creating publish graph client with default service name %s", service_name.c_str() );
        }
        for( const auto& robot_name : robot_names_ ) {
            service_name = "/" + robot_name + "/publish_graph";
            RCLCPP_INFO( this->get_logger(), "Creating publish graph client for robot %s with service name", service_name.c_str() );
            publish_graph_clients_[robot_name] = this->create_client<vamex_slam_msgs::srv::PublishGraph>( service_name );
        }

        // wait for service servers to start
        for( const auto& robot_name : robot_names_ ) {
            while( !publish_graph_clients_[robot_name]->wait_for_service( std::chrono::seconds( 1 ) ) ) {
                if( !rclcpp::ok() ) {
                    RCLCPP_ERROR( this->get_logger(), "Interrupted while waiting for the service. Exiting." );
                    rclcpp::shutdown();
                }
                RCLCPP_INFO_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 500,
                                             "Waiting for publish graph service for " << robot_name );
            }
        }

        // Set the distances from each robot to other robots to a negative value to indicate that they are not known yet
        for( size_t i = 0; i < robot_names_.size() - 1; i++ ) {
            for( size_t j = i + 1; j < robot_names_.size() - 1; j++ ) {
                // robot_distances_[std::make_pair( robot_names_[i], robot_names_[j] )] = -1.0;
                robot_distances_.insert( std::make_pair( std::make_pair( robot_names_[i], robot_names_[j] ), -1.0 ) );
            }
        }

        odom_broadcast_sub_ = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
            "/hdl_graph_slam/odom_broadcast", rclcpp::QoS( 100 ),
            std::bind( &MultiRobotCommunicator::odom_broadcast_callback, this, std::placeholders::_1 ) );

        // Initialiaze the tf2 buffer and listener to get odom 2 map transforms
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>( this->get_clock() );
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );
    }

    void odom_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr odom_msg )
    {
        // update the pose for this robot
        robot_odom2baselink_transforms_[odom_msg->robot_name] = pose2isometry( odom_msg->pose );
        RCLCPP_INFO_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 5000,
                                     "Received odom broadcast for " << odom_msg->robot_name << " with pose\n"
                                                                    << pose2isometry( odom_msg->pose ).matrix() );

        // update the odom2map transforms available on tf2
        update_odom2map_transforms();

        // update the distances to all robots
        update_distances();
    }

    void update_odom2map_transforms()
    {
        // TODO add map frame and odom frame as member variables
        for( const auto& robot_name : robot_names_ ) {
            if( !tf_buffer_->canTransform( robot_name + "/map", robot_name + "/odom", rclcpp::Time( 0 ) ) ) {
                RCLCPP_WARN( this->get_logger(), "Cannot transform %s to %s", ( robot_name + "/map" ).c_str(),
                             ( robot_name + "/odom" ).c_str() );
                continue;
            }

            geometry_msgs::msg::TransformStamped t_odom2map;
            try {
                t_odom2map = tf_buffer_->lookupTransform( robot_name + "/map", robot_name + "/odom", rclcpp::Time( 0 ) );
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
        for( size_t i = 0; i < robot_names_.size() - 1; i++ ) {
            for( size_t j = i + 1; j < robot_names_.size() - 1; j++ ) {
                // Transform the pose of both robots to the map frame of robot i
                Eigen::Vector3d robot_i_position_in_map_i =
                    ( robot_odom2map_transforms_[robot_names_[i]].second * robot_odom2baselink_transforms_[robot_names_[i]] ).translation();
                RCLCPP_INFO_STREAM( this->get_logger(), "robot_i_position_in_map_i: " << robot_i_position_in_map_i.transpose() );
                Eigen::Vector3d robot_j_position_in_map_j =
                    ( robot_odom2map_transforms_[robot_names_[j]].second * robot_odom2baselink_transforms_[robot_names_[j]] ).translation();
                RCLCPP_INFO_STREAM( this->get_logger(), "robot_j_position_in_map_j: " << robot_j_position_in_map_j.transpose() );
            }
        }
    }

private:
    // robot name -> publihsh graph client for that robot
    std::unordered_map<std::string, rclcpp::Client<vamex_slam_msgs::srv::PublishGraph>::SharedPtr> publish_graph_clients_;

    // robot name -> odom2baselink transform for that robot most recently received
    std::unordered_map<std::string, Eigen::Isometry3d> robot_odom2baselink_transforms_;

    // robot name -> odom2map transform for that robot most recently received
    std::unordered_map<std::string, std::pair<geometry_msgs::msg::Transform, Eigen::Isometry3d>> robot_odom2map_transforms_;

    // pair (robot name a, robot name b) -> distance between robot a and robot b
    std::unordered_map<std::pair<std::string, std::string>, double, PairHash> robot_distances_;

    // pair (robot name a, robot name b) -> last time the distance between robot a and robot b was used to call publish_graph
    std::unordered_map<std::pair<std::string, std::string>, Eigen::Isometry3d, PairHash> robot_last_publish_graph_poses_;


    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr odom_broadcast_sub_;

    std::vector<std::string> robot_names_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace hdl_graph_slam


int
main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    // TODO decide if multi_threaded_executor is needed
    auto node = std::make_shared<hdl_graph_slam::MultiRobotCommunicator>();
    rclcpp::spin( node );
    rclcpp::shutdown();

    return 0;
}