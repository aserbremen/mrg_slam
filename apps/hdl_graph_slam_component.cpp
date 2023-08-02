// SPDX-License-Identifier: BSD-2-Clause

#include <angles/angles.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vamex_slam_msgs/msg/graph_ros.hpp>
#include <vamex_slam_msgs/msg/pose_with_name.hpp>
#include <vamex_slam_msgs/msg/pose_with_name_array.hpp>
#include <vamex_slam_msgs/srv/dump_graph.hpp>
#include <vamex_slam_msgs/srv/get_graph_estimate.hpp>
#include <vamex_slam_msgs/srv/get_graph_gids.hpp>
#include <vamex_slam_msgs/srv/get_map.hpp>
#include <vamex_slam_msgs/srv/publish_graph.hpp>
#include <vamex_slam_msgs/srv/request_graphs.hpp>
#include <vamex_slam_msgs/srv/save_map.hpp>
// #include <pcl_ros/point_cloud.h>
// #include <pluginlib/class_list_macros.h>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Time.h>
// #include <tf_conversions/tf_eigen.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <hdl_graph_slam/DumpGraph.h>
// #include <hdl_graph_slam/GetGraphEstimate.h>
// #include <hdl_graph_slam/GetMap.h>
// #include <hdl_graph_slam/GraphRos.h>
// #include <hdl_graph_slam/PoseWithName.h>
// #include <hdl_graph_slam/PoseWithNameArray.h>
// #include <hdl_graph_slam/PublishGraph.h>
// #include <hdl_graph_slam/SaveMap.h>
// #include <nav_msgs/Odometry.h>
// #include <nodelet/nodelet.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <hdl_graph_slam/edge.hpp>
#include <hdl_graph_slam/floor_coeffs_processor.hpp>
#include <hdl_graph_slam/global_id.hpp>
#include <hdl_graph_slam/gps_processor.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/imu_processor.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/markers_publisher.hpp>
#include <hdl_graph_slam/ros_utils.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

// TODO add visbility control? https://gcc.gnu.org/wiki/Visibility
// #include "hdl_graph_slam/visibility_control.h"

// TODO go over all callbacks and decide whether to pass const msg::SharedPtr or ConstSharedPtr

namespace hdl_graph_slam {
// class HdlGraphSlamNodelet : public nodelet::Nodelet {
class HdlGraphSlamComponent : public rclcpp::Node {
public:
    // TODO add visibility control? https://gcc.gnu.org/wiki/Visibility
    // COMPOSITION_PUBLIC
    typedef pcl::PointXYZI                                                                                          PointT;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> ApproxSyncPolicy;

    // HdlGraphSlamComponent() {}
    HdlGraphSlamComponent( const rclcpp::NodeOptions            &options,
                           const std::vector<rclcpp::Parameter> &_param_vec = std::vector<rclcpp::Parameter>() ) :
        Node( "hdl_graph_slam_component", options ), param_vec( _param_vec )
    {
        // Since we need to pass the shared pointer from this node to other classes and functions, we start a one-shot timer to call the
        // onInit() method
        one_shot_initalization_timer = this->create_wall_timer( std::chrono::milliseconds( 100 ),
                                                                std::bind( &HdlGraphSlamComponent::onInit, this ) );
    }
    virtual ~HdlGraphSlamComponent() {}

    // Initialize the component
    virtual void onInit()
    {
        RCLCPP_INFO( this->get_logger(), "Initializing hdl_graph_slam component..." );
        // Deactivate this timer immediately so the initialization is only performed once
        one_shot_initalization_timer->cancel();

        initialize_params( param_vec );

        // Get the shared pointer to the ROS2 node to pass it to other classes and functions
        auto node_ros = shared_from_this();

        // Initialize variables
        trans_odom2map.setIdentity();
        anchor_node = nullptr;
        anchor_edge = nullptr;

        graph_slam.reset( new GraphSLAM( g2o_solver_type ) );
        graph_slam->set_save_graph( save_graph );

        keyframe_updater.reset( new KeyframeUpdater( node_ros ) );
        loop_detector.reset( new LoopDetector( node_ros ) );
        map_cloud_generator.reset( new MapCloudGenerator() );
        inf_calclator.reset( new InformationMatrixCalculator( node_ros ) );

        gid_generator = std::unique_ptr<GlobalIdGenerator>( new GlobalIdGenerator( node_ros, own_name, robot_names ) );

        // subscribers
        if( !model_namespace.empty() ) {
            odom_sub_topic  = "/" + model_namespace + "/scan_matching_odometry" + odom_sub_topic;
            cloud_sub_topic = "/" + model_namespace + "/prefiltering" + cloud_sub_topic;
        }
        RCLCPP_INFO( this->get_logger(), "Subscribing to odom topic %s", odom_sub_topic.c_str() );
        RCLCPP_INFO( this->get_logger(), "Subscribing to cloud topic %s", cloud_sub_topic.c_str() );
        auto qos  = rmw_qos_profile_default;
        qos.depth = 256;
        odom_sub.subscribe( node_ros, odom_sub_topic, qos );
        qos.depth = 32;
        cloud_sub.subscribe( node_ros, cloud_sub_topic, qos );
        // sync.reset( new message_filters::Synchronizer<ApproxSyncPolicy>( ApproxSyncPolicy( 32 ), *odom_sub, *cloud_sub ) );
        sync.reset( new message_filters::Synchronizer<ApproxSyncPolicy>( ApproxSyncPolicy( 32 ), odom_sub, cloud_sub ) );
        // sync->registerCallback( boost::bind( &HdlGraphSlamComponent::cloud_callback, this, _1, _2 ) );
        sync->registerCallback( std::bind( &HdlGraphSlamComponent::cloud_callback, this, std::placeholders::_1, std::placeholders::_2 ) );

        odom_broadcast_sub = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
            "/hdl_graph_slam/odom_broadcast", rclcpp::QoS( 100 ),
            std::bind( &HdlGraphSlamComponent::odom_broadcast_callback, this, std::placeholders::_1 ) );

        // Use a reentrant callbackgroup for odom_broadcast_sub to avoid deadlock, enabling the publish graph service to be called from the
        // same thread as the slam_pose_broadcast_callback
        rclcpp::SubscriptionOptions      sub_options;
        rclcpp::CallbackGroup::SharedPtr reentrant_callback_group = this->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
        sub_options.callback_group                                = reentrant_callback_group;
        slam_pose_broadcast_sub                                   = this->create_subscription<vamex_slam_msgs::msg::PoseWithName>(
            "/hdl_graph_slam/slam_pose_broadcast", rclcpp::QoS( 100 ),
            std::bind( &HdlGraphSlamComponent::slam_pose_broadcast_callback, this, std::placeholders::_1 ), sub_options );
        for( const auto &robot_name : robot_names ) {
            if( robot_name != own_name ) {
                request_graph_service_clients[robot_name] = this->create_client<vamex_slam_msgs::srv::PublishGraph>(
                    "/" + robot_name + "/hdl_graph_slam/publish_graph", rmw_qos_profile_services_default, reentrant_callback_group );
                others_last_accum_dist[robot_name] = -1.0;
            }
        }


        if( init_pose_topic != "NONE" ) {
            init_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>( init_pose_topic, rclcpp::QoS( 32 ),
                                                                                std::bind( &HdlGraphSlamComponent::init_pose_callback, this,
                                                                                           std::placeholders::_1 ) );
        }

        // publishers
        odom2map_pub            = this->create_publisher<geometry_msgs::msg::TransformStamped>( "/hdl_graph_slam/odom2pub", 16 );
        map_points_pub          = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/hdl_graph_slam/map_points", rclcpp::QoS( 1 ) );
        read_until_pub          = this->create_publisher<std_msgs::msg::Header>( "/hdl_graph_slam/read_until", rclcpp::QoS( 16 ) );
        odom_broadcast_pub      = this->create_publisher<vamex_slam_msgs::msg::PoseWithName>( "/hdl_graph_slam/odom_broadcast",
                                                                                         rclcpp::QoS( 16 ) );
        slam_pose_broadcast_pub = this->create_publisher<vamex_slam_msgs::msg::PoseWithName>( "/hdl_graph_slam/slam_pose_broadcast",
                                                                                              rclcpp::QoS( 16 ) );
        others_poses_pub        = this->create_publisher<vamex_slam_msgs::msg::PoseWithNameArray>( "/hdl_graph_slam/others_poses",
                                                                                            rclcpp::QoS( 16 ) );

        // We need to define a special function to pass arguments to a ROS2 callback with multiple parameters when the callback is a class
        // member function, see https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
        // If these service callbacks dont work during runtime, try using lambda functions as in
        // https://github.com/ros2/demos/blob/foxy/demo_nodes_cpp/src/services/add_two_ints_server.cpp
        // Dumb service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::DumpGraph::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::DumpGraph::Response>      res )>
            dump_service_callback = std::bind( &HdlGraphSlamComponent::dump_service, this, std::placeholders::_1, std::placeholders::_2 );
        dump_service_server       = this->create_service<vamex_slam_msgs::srv::DumpGraph>( "/hdl_graph_slam/dump", dump_service_callback );
        // Save map service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::SaveMap::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::SaveMap::Response>      res )>
            save_map_service_callback = std::bind( &HdlGraphSlamComponent::save_map_service, this, std::placeholders::_1,
                                                   std::placeholders::_2 );
        save_map_service_server       = this->create_service<vamex_slam_msgs::srv::SaveMap>( "/hdl_graph_slam/save_map",
                                                                                       save_map_service_callback );
        // Get map service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::GetMap::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::GetMap::Response>      res )>
            get_map_service_callback = std::bind( &HdlGraphSlamComponent::get_map_service, this, std::placeholders::_1,
                                                  std::placeholders::_2 );
        get_map_service_server = this->create_service<vamex_slam_msgs::srv::GetMap>( "/hdl_graph_slam/get_map", get_map_service_callback );
        // Get graph estimate service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::GetGraphEstimate::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::GetGraphEstimate::Response>      res )>
            get_graph_estimate_service_callback = std::bind( &HdlGraphSlamComponent::get_graph_estimate_service, this,
                                                             std::placeholders::_1, std::placeholders::_2 );
        get_graph_estimate_service_server       = this->create_service<vamex_slam_msgs::srv::GetGraphEstimate>(
            "/hdl_graph_slam/get_graph_estimate", get_graph_estimate_service_callback );
        // Publish graph service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::PublishGraph::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::PublishGraph::Response>      res )>
            publish_graph_service_callback = std::bind( &HdlGraphSlamComponent::publish_graph_service, this, std::placeholders::_1,
                                                        std::placeholders::_2 );
        publish_graph_service_server       = this->create_service<vamex_slam_msgs::srv::PublishGraph>( "/hdl_graph_slam/publish_graph",
                                                                                                 publish_graph_service_callback );
        // Request graph service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::RequestGraphs::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::RequestGraphs::Response>      res )>
            request_graph_service_callback = std::bind( &HdlGraphSlamComponent::request_graph_service, this, std::placeholders::_1,
                                                        std::placeholders::_2 );
        request_graph_service_server       = this->create_service<vamex_slam_msgs::srv::RequestGraphs>( "/hdl_graph_slam/request_graph",
                                                                                                  request_graph_service_callback );
        // Get graph IDs (gids) service
        std::function<void( const std::shared_ptr<vamex_slam_msgs::srv::GetGraphGids::Request> req,
                            std::shared_ptr<vamex_slam_msgs::srv::GetGraphGids::Response>      res )>
            get_graph_gids_service_callback = std::bind( &HdlGraphSlamComponent::get_graph_gids_service, this, std::placeholders::_1,
                                                         std::placeholders::_2 );
        get_graph_gids_service_server       = this->create_service<vamex_slam_msgs::srv::GetGraphGids>( "/hdl_graph_slam/get_graph_gids",
                                                                                                  get_graph_gids_service_callback );

        cloud_msg_update_required          = false;
        graph_estimate_msg_update_required = false;
        optimization_timer = rclcpp::create_timer( node_ros, node_ros->get_clock(), rclcpp::Duration( graph_update_interval, 0 ),
                                                   std::bind( &HdlGraphSlamComponent::optimization_timer_callback, this ) );
        map_publish_timer  = rclcpp::create_timer( node_ros, node_ros->get_clock(), rclcpp::Duration( map_cloud_update_interval, 0 ),
                                                   std::bind( &HdlGraphSlamComponent::map_points_publish_timer_callback, this ) );

        gps_processor.onInit( node_ros );
        imu_processor.onInit( node_ros );
        floor_coeffs_processor.onInit( node_ros );
        markers_pub.onInit( node_ros );

        // Print the all parameters declared in this node so far
        print_ros2_parameters( this->get_node_parameters_interface(), this->get_logger() );
    }

private:
    void initialize_params( const std::vector<rclcpp::Parameter> &param_vec = std::vector<rclcpp::Parameter>() )
    {
        // Declare all parameters used by this class and its members first

        // General and scenario parameters
        this->declare_parameter<std::string>( "points_topic", "/velodyne_points" );
        this->declare_parameter<std::string>( "own_name", "atlas" );
        this->declare_parameter<std::vector<std::string>>( "/properties/scenario/rovers/names", { "atlas", "bestla" } );
        this->declare_parameter<std::string>( "model_namespace", "" );
        this->declare_parameter<std::string>( "odom_sub_topic", "/odom" );
        this->declare_parameter<std::string>( "cloud_sub_topic", "/filtered_points" );

        // Map parameters
        this->declare_parameter<std::string>( "map_frame_id", "map" );
        this->declare_parameter<std::string>( "odom_frame_id", "odom" );
        this->declare_parameter<double>( "map_cloud_resolution", 0.05 );
        this->declare_parameter<int>( "map_cloud_count_threshold", 2 );

        // Initial pose parameters
        this->declare_parameter<std::string>( "init_pose_topic", "NONE" );
        this->declare_parameter<std::vector<double>>( "init_pose", std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } );

        // Removing points of other robots from point cloud
        this->declare_parameter<double>( "robot_remove_points_radius", 2.0 );

        // GraphSLAM parameters
        this->declare_parameter<bool>( "fix_first_node", false );
        this->declare_parameter<bool>( "fix_first_node_adaptive", false );
        this->declare_parameter<std::vector<double>>( "fix_first_node_stddev",
                                                      std::vector<double>{ 0.5, 0.5, 0.5, angles::from_degrees( 5 ),
                                                                           angles::from_degrees( 5 ), angles::from_degrees( 5 ) } );
        this->declare_parameter<int>( "max_keyframes_per_update", 10 );
        this->declare_parameter<std::string>( "odometry_edge_robust_kernel", "NONE" );
        this->declare_parameter<double>( "odometry_edge_robust_kernel_size", 1.0 );
        this->declare_parameter<std::string>( "loop_closure_edge_robust_kernel", "Huber" );
        this->declare_parameter<double>( "loop_closure_edge_robust_kernel_size", 1.0 );
        this->declare_parameter<std::string>( "g2o_solver_type", "lm_var_cholmod" );
        this->declare_parameter<bool>( "save_graph", true );
        this->declare_parameter<int>( "g2o_solver_num_iterations", 1024 );
        this->declare_parameter<double>( "graph_update_interval", 3.0 );
        this->declare_parameter<double>( "map_cloud_update_interval", 10.0 );
        this->declare_parameter<double>( "graph_request_min_accum_dist", 3.0 );
        this->declare_parameter<double>( "graph_request_max_robot_dist", 10.0 );

        // KeyframeUpdater parameters (not directly used by this class)
        this->declare_parameter<double>( "keyframe_delta_trans", 2.0 );
        this->declare_parameter<double>( "keyframe_delta_angle", 2.0 );

        // LoopDetector parameters (not directly used by this class)
        this->declare_parameter<double>( "distance_thresh", 5.0 );
        this->declare_parameter<double>( "accum_distance_thresh", 8.0 );
        this->declare_parameter<double>( "min_edge_interval", 5.0 );
        this->declare_parameter<double>( "fitness_score_max_range", std::numeric_limits<double>::max() );
        this->declare_parameter<double>( "fitness_score_thresh", 0.5 );
        this->declare_parameter<bool>( "use_planar_registration_guess", false );

        // Loop closure (scan matching regisration LoopDetector) parameters (not directly used by this class)
        this->declare_parameter<std::string>( "registration_method", "FAST_GICP" );
        this->declare_parameter<int>( "reg_num_threads", 0 );
        this->declare_parameter<double>( "reg_transformation_epsilon", 0.1 );
        this->declare_parameter<int>( "reg_maximum_iterations", 64 );
        this->declare_parameter<double>( "reg_max_correspondence_distance", 2.0 );
        this->declare_parameter<int>( "reg_max_optimizer_iterations", 20 );
        this->declare_parameter<bool>( "reg_use_reciprocal_correspondences", false );
        this->declare_parameter<int>( "reg_correspondence_randomness", 20 );
        this->declare_parameter<double>( "reg_resolution", 1.0 );
        this->declare_parameter<std::string>( "reg_nn_search_method", "DIRECT7" );

        // InformationMatrixCalculator parameters (not directly used by this class)
        this->declare_parameter<bool>( "use_const_inf_matrix", false );
        this->declare_parameter<double>( "const_stddev_x", 0.5 );
        this->declare_parameter<double>( "const_stddev_q", 0.1 );
        this->declare_parameter<double>( "var_gain_a", 20.0 );
        this->declare_parameter<double>( "min_stddev_x", 0.1 );
        this->declare_parameter<double>( "max_stddev_x", 5.0 );
        this->declare_parameter<double>( "min_stddev_q", 0.05 );
        this->declare_parameter<double>( "max_stddev_q", 0.2 );
        // this->declare_parameter<double>( "fitness_score_thresh", 0.5 ); // already declared for LoopDetector

        // GpsProcessor parameters (not directly used by this class)
        this->declare_parameter<bool>( "enable_gps", true );
        this->declare_parameter<double>( "gps_time_offset", 0.0 );
        this->declare_parameter<double>( "gps_edge_stddev_xy", 10000.0 );
        this->declare_parameter<double>( "gps_edge_stddev_z", 10.0 );
        this->declare_parameter<std::string>( "gps_edge_robust_kernel", "NONE" );
        this->declare_parameter<double>( "gps_edge_robust_kernel_size", 1.0 );

        // ImuProcessor parameters (not directly used by this class)
        this->declare_parameter<double>( "imu_time_offset", 0.0 );
        this->declare_parameter<bool>( "enable_imu_orientation", false );
        this->declare_parameter<bool>( "enable_imu_acceleration", false );
        this->declare_parameter<double>( "imu_orientation_edge_stddev", 0.1 );
        this->declare_parameter<double>( "imu_acceleration_edge_stddev", 3.0 );
        this->declare_parameter<std::string>( "imu_orientation_edge_robust_kernel", "NONE" );
        this->declare_parameter<std::string>( "imu_acceleration_edge_robust_kernel", "NONE" );
        this->declare_parameter<double>( "imu_orientation_edge_robust_kernel_size", 1.0 );
        this->declare_parameter<double>( "imu_acceleration_edge_robust_kernel_size", 1.0 );

        // FloorCoeffsProcessor parameters (not directly used by this class)
        this->declare_parameter<double>( "floor_edge_stddev", 10.0 );
        this->declare_parameter<std::string>( "floor_edge_robust_kernel", "NONE" );
        this->declare_parameter<double>( "floor_edge_robust_kernel_size", 1.0 );

        // Overwrite parameters if param_vec is provided, use case manual composition (debugging)
        if( !param_vec.empty() ) {
            this->set_parameters( param_vec );
        }

        // Set member variables for this class
        points_topic    = this->get_parameter( "points_topic" ).as_string();
        own_name        = this->get_parameter( "own_name" ).as_string();
        robot_names     = this->get_parameter( "/properties/scenario/rovers/names" ).as_string_array();
        model_namespace = this->get_parameter( "model_namespace" ).as_string();
        odom_sub_topic  = this->get_parameter( "odom_sub_topic" ).as_string();
        cloud_sub_topic = this->get_parameter( "cloud_sub_topic" ).as_string();

        map_frame_id              = this->get_parameter( "map_frame_id" ).as_string();
        odom_frame_id             = this->get_parameter( "odom_frame_id" ).as_string();
        map_cloud_resolution      = this->get_parameter( "map_cloud_resolution" ).as_double();
        map_cloud_count_threshold = this->get_parameter( "map_cloud_count_threshold" ).as_int();

        init_pose_topic = this->get_parameter( "init_pose_topic" ).as_string();
        init_pose_vec   = this->get_parameter( "init_pose" ).as_double_array();

        robot_remove_points_radius = static_cast<float>( this->get_parameter( "robot_remove_points_radius" ).as_double() );

        fix_first_node                       = this->get_parameter( "fix_first_node" ).as_bool();
        fix_first_node_adaptive              = this->get_parameter( "fix_first_node_adaptive" ).as_bool();
        fix_first_node_stddev_vec            = this->get_parameter( "fix_first_node_stddev" ).as_double_array();
        max_keyframes_per_update             = this->get_parameter( "max_keyframes_per_update" ).as_int();
        odometry_edge_robust_kernel          = this->get_parameter( "odometry_edge_robust_kernel" ).as_string();
        odometry_edge_robust_kernel_size     = this->get_parameter( "odometry_edge_robust_kernel_size" ).as_double();
        loop_closure_edge_robust_kernel      = this->get_parameter( "loop_closure_edge_robust_kernel" ).as_string();
        loop_closure_edge_robust_kernel_size = this->get_parameter( "loop_closure_edge_robust_kernel_size" ).as_double();
        g2o_solver_num_iterations            = this->get_parameter( "g2o_solver_num_iterations" ).as_int();
        g2o_solver_type                      = this->get_parameter( "g2o_solver_type" ).as_string();
        save_graph                           = this->get_parameter( "save_graph" ).as_bool();
        graph_update_interval                = this->get_parameter( "graph_update_interval" ).as_double();
        map_cloud_update_interval            = this->get_parameter( "map_cloud_update_interval" ).as_double();
        graph_request_min_accum_dist         = this->get_parameter( "graph_request_min_accum_dist" ).as_double();
        graph_request_max_robot_dist         = this->get_parameter( "graph_request_max_robot_dist" ).as_double();
    }

    /**
     * @brief received point clouds are pushed to #keyframe_queue
     * @param odom_msg
     * @param cloud_msg
     */
    // void cloud_callback( const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg )
    void cloud_callback( nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg )
    {
        // const ros::Time  &stamp = cloud_msg->header.stamp;
        const builtin_interfaces::msg::Time &stamp = cloud_msg->header.stamp;
        Eigen::Isometry3d                    odom  = odom2isometry( odom_msg );

        if( base_frame_id.empty() ) {
            base_frame_id = cloud_msg->header.frame_id;
        }

        bool   update_required = keyframe_updater->update( odom );
        double accum_d         = keyframe_updater->get_accum_distance();

        if( !update_required ) {
            if( keyframe_queue.empty() ) {
                std_msgs::msg::Header read_until;
                // read_until.stamp    =  stamp + ros::Duration( 10, 0 ); // ROS1
                read_until.stamp    = ( rclcpp::Time( stamp ) + rclcpp::Duration( 10, 0 ) ).operator builtin_interfaces::msg::Time();
                read_until.frame_id = points_topic;
                // read_until_pub.publish( read_until );
                read_until_pub->publish( read_until );
                read_until.frame_id = "/filtered_points";
                // read_until_pub.publish( read_until );
                read_until_pub->publish( read_until );
            }
        } else {
            pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
            pcl::fromROSMsg( *cloud_msg, *cloud );
            sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg_filtered = cloud_msg;

            // get poses of other robots and remove corresponding points
            Eigen::Isometry3d            map2odom;
            std::vector<Eigen::Vector3d> others_positions;
            {
                std::lock_guard<std::mutex> lock( trans_odom2map_mutex );
                // Weird bug where the sign of certain numbers is flipped
                // map2odom = trans_odom2map.inverse().cast<double>(); TODO check if this fails at other places
                if( trans_odom2map.isApprox( Eigen::Matrix4f::Identity() ) ) {
                    map2odom.setIdentity();
                } else {
                    trans_odom2map.inverse().cast<double>();
                }

                others_positions.resize( others_odom_poses.size() );
                size_t i = 0;
                for( const auto &odom_pose : others_odom_poses ) {
                    others_positions[i].x() = odom_pose.second.second.position.x;
                    others_positions[i].y() = odom_pose.second.second.position.y;
                    others_positions[i].z() = odom_pose.second.second.position.z;
                    i++;
                }
            }
            if( !others_positions.empty() ) {
                Eigen::Isometry3d map2sensor = odom.inverse() * map2odom;

                // transform other robots' positions to sensro frame
                std::vector<Eigen::Vector3f> others_positions_sensor;
                others_positions_sensor.resize( others_positions.size() );
                for( size_t i = 0; i < others_positions.size(); i++ ) {
                    others_positions_sensor[i] = ( map2sensor * others_positions[i] ).cast<float>();
                }

                float robot_radius_sqr = robot_remove_points_radius * robot_remove_points_radius;

                // get points to be removed
                pcl::PointIndices::Ptr to_be_removed( new pcl::PointIndices() );
                to_be_removed->indices.reserve( cloud->size() );
                for( size_t i = 0; i < cloud->size(); i++ ) {
                    const auto     &point_pcl = ( *cloud )[i];
                    Eigen::Vector3f point( point_pcl.x, point_pcl.y, point_pcl.z );
                    for( const auto &other_position_sensor : others_positions_sensor ) {
                        float distSqr = ( point - other_position_sensor ).squaredNorm();
                        if( distSqr < robot_radius_sqr ) {
                            to_be_removed->indices.push_back( i );
                            break;
                        }
                    }
                }

                // remove points
                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud( cloud );
                extract.setIndices( to_be_removed );
                extract.setNegative( true );
                extract.filter( *cloud );

                // create ROS cloud to be stored within the keyframe
                sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_tmp( new sensor_msgs::msg::PointCloud2 );
                pcl::toROSMsg( *cloud, *cloud_msg_tmp );
                // TODO does this work? as cloud_msg_filtered is assigned above as well
                cloud_msg_filtered = cloud_msg_tmp;
            }

            // create keyframe and add it to the queue
            KeyFrame::Ptr keyframe( new KeyFrame( stamp, odom, accum_d, cloud, cloud_msg_filtered ) );

            std::lock_guard<std::mutex> lock( keyframe_queue_mutex );
            keyframe_queue.push_back( keyframe );
        }

        // publish own odometry
        vamex_slam_msgs::msg::PoseWithName pose_msg;
        pose_msg.header     = odom_msg->header;
        pose_msg.robot_name = own_name;
        pose_msg.pose       = odom_msg->pose.pose;
        pose_msg.accum_dist = accum_d;
        // odom_broadcast_pub.publish( pose_msg );
        odom_broadcast_pub->publish( pose_msg );
    }

    /**
     * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
     * @return if true, at least one keyframe was added to the pose graph
     */
    bool flush_keyframe_queue()
    {
        std::lock_guard<std::mutex> lock( keyframe_queue_mutex );

        if( keyframe_queue.empty() ) {
            return false;
        }

        if( keyframes.empty() && new_keyframes.empty() ) {
            // init pose
            Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
            if( init_pose != nullptr ) {
                Eigen::Isometry3d pose;
                // tf::poseMsgToEigen( init_pose->pose.pose, pose );
                tf2::fromMsg( init_pose->pose.pose, pose );
                pose = pose * keyframe_queue[0]->odom.inverse();  // "remove" odom (which will be added later again) such that the init
                                                                  // pose actually corresponds to the received pose
                pose_mat = pose.matrix();
            } else {
                Eigen::Matrix<double, 6, 1> p( init_pose_vec.data() );
                pose_mat.topLeftCorner<3, 3>() = ( Eigen::AngleAxisd( p[5], Eigen::Vector3d::UnitX() )
                                                   * Eigen::AngleAxisd( p[4], Eigen::Vector3d::UnitY() )
                                                   * Eigen::AngleAxisd( p[3], Eigen::Vector3d::UnitZ() ) )
                                                     .toRotationMatrix();
                pose_mat.topRightCorner<3, 1>() = p.head<3>();
                // don't remove odom because we assume that the provided pose corresponds to the pose of the rover when starting the
                // system
            }
            RCLCPP_INFO_STREAM( this->get_logger(), "initial pose:\n" << pose_mat );
            trans_odom2map_mutex.lock();
            trans_odom2map = pose_mat.cast<float>();
            trans_odom2map_mutex.unlock();
        }

        trans_odom2map_mutex.lock();
        Eigen::Isometry3d odom2map( trans_odom2map.cast<double>() );
        trans_odom2map_mutex.unlock();

        int num_processed = 0;
        for( int i = 0; i < std::min<int>( keyframe_queue.size(), max_keyframes_per_update ); i++ ) {
            num_processed = i;

            const auto &keyframe = keyframe_queue[i];
            // new_keyframes will be tested later for loop closure
            new_keyframes.push_back( keyframe );

            // add pose node
            Eigen::Isometry3d odom = odom2map * keyframe->odom;
            keyframe->node         = graph_slam->add_se3_node( odom );
            keyframe->set_gid( *gid_generator );
            keyframe_gids[keyframe->gid]   = keyframe;
            keyframe_hash[keyframe->stamp] = keyframe;

            // first keyframe?
            if( keyframes.empty() && new_keyframes.size() == 1 ) {
                keyframe->exclude_from_map = true;  // exclude point cloud of first keyframe from map, because points corresponding to
                                                    // other robots have not been filtered for this keyframe

                // fix the first node
                // if( private_nh.param<bool>( "fix_first_node", false ) ) {
                if( fix_first_node ) {
                    Eigen::MatrixXd information = Eigen::MatrixXd::Identity( 6, 6 );
                    for( int i = 0; i < 6; i++ ) {
                        information( i, i ) = 1.0 / ( fix_first_node_stddev_vec[i] * fix_first_node_stddev_vec[i] );
                    }
                    RCLCPP_INFO_STREAM( this->get_logger(), "fixing first node with information:\n" << information );

                    anchor_node = graph_slam->add_se3_node( Eigen::Isometry3d::Identity() );
                    anchor_node->setFixed( true );
                    // KeyFrame::Ptr anchor_kf( new KeyFrame( ros::Time(), Eigen::Isometry3d::Identity(), -1, nullptr ) );
                    KeyFrame::Ptr anchor_kf( new KeyFrame( rclcpp::Time(), Eigen::Isometry3d::Identity(), -1, nullptr ) );
                    // if( !private_nh.param<bool>( "fix_first_node_adaptive", true ) ) {
                    if( !fix_first_node_adaptive ) {
                        anchor_kf->gid = 0;  // if anchor node is not adaptive (i.e. stays at the origin), its GID needs to be 0 for all
                                             // robots
                    }
                    anchor_kf->node  = anchor_node;
                    keyframe_gids[0] = anchor_kf;

                    anchor_edge = graph_slam->add_se3_edge( anchor_node, keyframe->node, keyframe->node->estimate(), information );
                    auto edge   = new Edge( anchor_edge, Edge::TYPE_ODOM, (GlobalId)0, keyframe->gid, *gid_generator );
                    edges.emplace_back( edge );
                    edge_gids.insert( edge->gid );
                }
            }

            if( i == 0 && keyframes.empty() ) {
                prev_robot_keyframe = keyframe;
                continue;
            }

            // add edge between consecutive keyframes
            Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_robot_keyframe->odom;
            Eigen::MatrixXd   information   = inf_calclator->calc_information_matrix( keyframe->cloud, prev_robot_keyframe->cloud,
                                                                                      relative_pose );
            auto graph_edge = graph_slam->add_se3_edge( keyframe->node, prev_robot_keyframe->node, relative_pose, information );
            auto edge       = new Edge( graph_edge, Edge::TYPE_ODOM, keyframe->gid, prev_robot_keyframe->gid, *gid_generator );
            edges.emplace_back( edge );
            edge_gids.insert( edge->gid );
            // graph_slam->add_robust_kernel( graph_edge, private_nh.param<std::string>( "odometry_edge_robust_kernel", "NONE" ),
            //                                private_nh.param<double>( "odometry_edge_robust_kernel_size", 1.0 ) );
            graph_slam->add_robust_kernel( graph_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
            prev_robot_keyframe = keyframe;
        }

        // std_msgs::Header read_until;
        std_msgs::msg::Header read_until;
        // read_until.stamp    = keyframe_queue[num_processed]->stamp + ros::Duration( 10, 0 );
        read_until.stamp =
            ( rclcpp::Time( keyframe_queue[num_processed]->stamp ) + rclcpp::Duration( 10, 0 ) ).operator builtin_interfaces::msg::Time();
        read_until.frame_id = points_topic;
        // read_until_pub.publish( read_until );
        read_until_pub->publish( read_until );
        read_until.frame_id = "/filtered_points";
        // read_until_pub.publish( read_until );
        read_until_pub->publish( read_until );

        keyframe_queue.erase( keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1 );
        return true;
    }

    /**
     * @brief received graph from other robots are added to #graph_queue
     * @param graph_msg
     */
    // void graph_callback( const hdl_graph_slam::GraphRos::ConstPtr &graph_msg )
    void graph_callback( vamex_slam_msgs::msg::GraphRos::ConstSharedPtr graph_msg )
    {
        std::lock_guard<std::mutex> lock( graph_queue_mutex );
        if( graph_msg->robot_name != own_name ) {
            graph_queue.push_back( graph_msg );
        }
    }


    /**
     * @brief all edges and keyframes from #graph_queue that are not known yet are added to the graph
     * @return if true, at least one edge or keyframe is added to the pose graph
     */
    bool flush_graph_queue()
    {
        std::lock_guard<std::mutex> lock( graph_queue_mutex );

        if( graph_queue.empty() || keyframes.empty() ) {
            return false;
        }

        // ROS_INFO_STREAM( "Received graph msgs: " << graph_queue.size() );
        RCLCPP_INFO_STREAM( this->get_logger(), "Received graph msgs: " << graph_queue.size() );

        // std::unordered_map<GlobalId, const KeyFrameRos *> unique_keyframes;
        // std::unordered_map<GlobalId, const EdgeRos *>     unique_edges;
        std::unordered_map<GlobalId, const vamex_slam_msgs::msg::KeyFrameRos *> unique_keyframes;
        std::unordered_map<GlobalId, const vamex_slam_msgs::msg::EdgeRos *>     unique_edges;

        std::unordered_map<std::string, std::pair<const GlobalId *, const geometry_msgs::msg::Pose *>> latest_robot_keyframes;

        for( const auto &graph_msg : graph_queue ) {
            for( const auto &edge_ros : graph_msg->edges ) {
                if( edge_gids.find( edge_ros.gid ) != edge_gids.end() ) {
                    continue;
                }
                if( edge_ignore_gids.find( edge_ros.gid ) != edge_ignore_gids.end() ) {
                    continue;
                }
                auto iter = unique_edges.find( edge_ros.gid );
                if( iter != unique_edges.end() ) {
                    iter->second = &edge_ros;
                } else {
                    unique_edges[edge_ros.gid] = &edge_ros;
                }
            }

            for( const auto &keyframe_ros : graph_msg->keyframes ) {
                if( keyframe_gids.find( keyframe_ros.gid ) != keyframe_gids.end() ) {
                    continue;
                }
                auto iter = unique_keyframes.find( keyframe_ros.gid );
                if( iter != unique_keyframes.end() ) {
                    iter->second = &keyframe_ros;
                } else {
                    unique_keyframes[keyframe_ros.gid] = &keyframe_ros;
                }
            }

            latest_robot_keyframes[graph_msg->robot_name] = std::make_pair<const GlobalId *, const geometry_msgs::msg::Pose *>(
                &graph_msg->latest_keyframe_gid, &graph_msg->latest_keyframe_odom );
        }

        // ROS_INFO_STREAM( "Unique keyframes: " << unique_keyframes.size() );
        // ROS_INFO_STREAM( "Unique edges:     " << unique_edges.size() );
        RCLCPP_INFO_STREAM( this->get_logger(), "Unique keyframes: " << unique_keyframes.size() );
        for( auto const &kf : unique_keyframes ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "Keyframe ID: " << kf.first );
        }
        RCLCPP_INFO_STREAM( this->get_logger(), "Unique edges:     " << unique_edges.size() );
        for( auto const &edge : unique_edges ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "Edge ID: " << static_cast<uint64_t>( edge.first ) << " from ID "
                                                                << edge.second->from_gid << " to ID " << edge.second->to_gid );
        }

        if( unique_keyframes.empty() || unique_edges.empty() ) {
            graph_queue.clear();
            return false;
        }

        for( const auto &kf : unique_keyframes ) {
            const auto &keyframe_ros = *kf.second;

            // ROS_INFO_STREAM( "Adding keyframe: " << keyframe_ros.gid );
            RCLCPP_INFO_STREAM( this->get_logger(), "Adding keyframe: " << keyframe_ros.gid );

            pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
            pcl::fromROSMsg( keyframe_ros.cloud, *cloud );
            // ros::Time stamp(keyframe_ros.stamp);
            // sensor_msgs::PointCloud2::Ptr cloud_ros = boost::make_shared<sensor_msgs::PointCloud2>( keyframe_ros.cloud );
            sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros = std::make_shared<sensor_msgs::msg::PointCloud2>( keyframe_ros.cloud );
            KeyFrame::Ptr keyframe( new KeyFrame( keyframe_ros.stamp, Eigen::Isometry3d::Identity(), -1, cloud, cloud_ros ) );

            Eigen::Isometry3d pose;
            // tf::poseMsgToEigen( keyframe_ros.estimate, pose );
            tf2::fromMsg( keyframe_ros.estimate, pose );
            keyframe->node               = graph_slam->add_se3_node( pose );
            keyframe->gid                = keyframe_ros.gid;
            keyframe->exclude_from_map   = keyframe_ros.exclude_from_map;
            keyframe_gids[keyframe->gid] = keyframe;
            new_keyframes.push_back( keyframe );  // new_keyframes will be tested later for loop closure
                                                  // don't add it to keyframe_hash, which is only used for floor_coeffs
                                                  // keyframe_hash[keyframe->stamp] = keyframe;
        }

        for( const auto &e : unique_edges ) {
            const auto &edge_ros      = *e.second;
            const auto &from_keyframe = keyframe_gids[edge_ros.from_gid];
            const auto &to_keyframe   = keyframe_gids[edge_ros.to_gid];

            if( from_keyframe->edge_exists( *to_keyframe, this->get_logger() ) ) {
                edge_ignore_gids.insert( edge_ros.gid );
                continue;
            }

            // ROS_INFO_STREAM( "Adding edge: " << edge_ros.gid << " (" << edge_ros.from_gid << " -> " << edge_ros.to_gid << ")" );
            RCLCPP_INFO_STREAM( this->get_logger(),
                                "Adding edge: " << edge_ros.gid << " (" << edge_ros.from_gid << " -> " << edge_ros.to_gid << ")" );

            Eigen::Isometry3d relpose;
            // tf::poseMsgToEigen( edge_ros.relative_pose, relpose );
            tf2::fromMsg( edge_ros.relative_pose, relpose );
            Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> information( edge_ros.information.data() );
            auto graph_edge = graph_slam->add_se3_edge( from_keyframe->node, to_keyframe->node, relpose, information );
            auto edge       = new Edge( graph_edge, edge_ros.type == 0 ? Edge::TYPE_ODOM : Edge::TYPE_LOOP );
            edge->gid       = edge_ros.gid;
            edge->from_gid  = edge_ros.from_gid;
            edge->to_gid    = edge_ros.to_gid;
            edges.emplace_back( edge );
            edge_gids.insert( edge->gid );

            if( edge->type == Edge::TYPE_ODOM ) {
                // graph_slam->add_robust_kernel( graph_edge, private_nh.param<std::string>( "odometry_edge_robust_kernel", "NONE" ),
                //                                private_nh.param<double>( "odometry_edge_robust_kernel_size", 1.0 ) );
                graph_slam->add_robust_kernel( graph_edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size );
            } else {
                // graph_slam->add_robust_kernel( graph_edge, private_nh.param<std::string>( "loop_closure_edge_robust_kernel", "NONE" ),
                //                                private_nh.param<double>( "loop_closure_edge_robust_kernel_size", 1.0 ) );
                graph_slam->add_robust_kernel( graph_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
            }
        }

        for( const auto &latest_keyframe : latest_robot_keyframes ) {
            auto &kf = others_prev_robot_keyframes[latest_keyframe.first];
            kf.first = keyframe_gids[*latest_keyframe.second.first];  // pointer to keyframe
            // tf::poseMsgToEigen( *latest_keyframe.second.second, kf.second );  // odometry
            tf2::fromMsg( *latest_keyframe.second.second, kf.second );  // odometry
        }

        graph_queue.clear();
        return true;
    }


    void update_pose( const Eigen::Isometry3d &odom2map, std::pair<Eigen::Isometry3d, geometry_msgs::msg::Pose> &odom_pose )
    {
        auto             &odom     = odom_pose.first;
        auto             &pose     = odom_pose.second;
        Eigen::Isometry3d new_pose = odom2map * odom;
        // tf::poseEigenToMsg( new_pose, pose );
        tf2::fromMsg( pose, new_pose );
    }


    void publish_slam_pose_broadcast( hdl_graph_slam::KeyFrame::ConstPtr kf )
    {
        vamex_slam_msgs::msg::PoseWithName slam_pose_msg;
        slam_pose_msg.header.stamp    = this->now();
        slam_pose_msg.header.frame_id = map_frame_id;
        slam_pose_msg.pose            = isometry2pose( kf->node->estimate() );
        slam_pose_msg.robot_name      = own_name;
        slam_pose_msg.accum_dist      = kf->accum_distance;
        slam_pose_broadcast_pub->publish( slam_pose_msg );
    }

    void slam_pose_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg )
    {
        if( slam_pose_msg->robot_name == own_name || prev_robot_keyframe == nullptr ) {
            return;
        }

        std::unique_lock<std::mutex> unique_lck( main_thread_mutex );

        Eigen::Vector3d own_position    = prev_robot_keyframe->estimate().translation();
        const auto     &robot_name      = slam_pose_msg->robot_name;
        double          accum_dist      = slam_pose_msg->accum_dist;
        double         &last_accum_dist = others_last_accum_dist[robot_name];
        others_slam_poses[robot_name]   = *slam_pose_msg;
        if( last_accum_dist >= 0 && fabs( accum_dist - last_accum_dist ) < graph_request_min_accum_dist ) {
            return;
        }

        double          max_robot_dist_sqr = graph_request_max_robot_dist * graph_request_max_robot_dist;
        Eigen::Vector3d other_position     = Eigen::Vector3d( slam_pose_msg->pose.position.x, slam_pose_msg->pose.position.y,
                                                              slam_pose_msg->pose.position.z );
        if( ( own_position - other_position ).squaredNorm() > max_robot_dist_sqr ) {
            return;
        }

        while( !request_graph_service_clients[robot_name]->wait_for_service( std::chrono::seconds( 2 ) ) ) {
            if( !rclcpp::ok() ) {
                return;
            }
            RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 1000,
                                         "Waiting for service " << request_graph_service_clients[robot_name]->get_service_name()
                                                                << " to appear..." );
        }

        RCLCPP_INFO_STREAM( this->get_logger(), "Requesting graph from rover " << robot_name << " with new accum dist "
                                                                               << accum_dist - last_accum_dist << "m" );
        vamex_slam_msgs::srv::PublishGraph::Request::SharedPtr req = std::make_shared<vamex_slam_msgs::srv::PublishGraph::Request>();

        req->robot_name = own_name;
        req->processed_keyframe_gids.reserve( keyframes.size() );
        for( const auto &keyframe : keyframes ) {
            req->processed_keyframe_gids.push_back( keyframe->gid );
        }
        req->processed_edge_gids.reserve( edges.size() );
        for( const auto &edge : edges ) {
            req->processed_edge_gids.push_back( edge->gid );
        }

        unique_lck.unlock();

        auto               result_future = request_graph_service_clients[robot_name]->async_send_request( req );
        std::future_status status        = result_future.wait_for( std::chrono::seconds( 5 ) );

        if( status == std::future_status::timeout ) {
            RCLCPP_WARN_STREAM( this->get_logger(), "Request graph service call to rover " << robot_name << " timed out" );
            return;
        }
        if( status == std::future_status::ready ) {
            RCLCPP_INFO_STREAM( this->get_logger(), "Request graph service call to rover " << robot_name << " successful" );
        }
        auto result = result_future.get();
        // Fill the graph queue with the received graph
        std::lock_guard<std::mutex> lock( graph_queue_mutex );
        graph_queue.push_back( std::make_shared<vamex_slam_msgs::msg::GraphRos>( std::move( result->graph ) ) );

        last_accum_dist = accum_dist;
    }

    /**
     * @brief received odom msgs from other robots proccessed
     * @param graph_msg
     */
    // void odom_broadcast_callback( const hdl_graph_slam::PoseWithName::ConstPtr &pose_msg )
    void odom_broadcast_callback( vamex_slam_msgs::msg::PoseWithName::ConstSharedPtr pose_msg )
    {
        if( pose_msg->robot_name == own_name ) {
            return;
        }

        // PoseWithNameArray pose_array_msg;
        // TODO it seems that other poses are not updated and the first frame is published all the time, fix?
        vamex_slam_msgs::msg::PoseWithNameArray pose_array_msg;
        pose_array_msg.header.stamp    = pose_msg->header.stamp;
        pose_array_msg.header.frame_id = map_frame_id;

        Eigen::Vector3d other_position, own_position;
        std::string     other_name = pose_msg->robot_name;

        // update poses of other robots in own map frame
        {
            std::lock_guard<std::mutex> lock( trans_odom2map_mutex );
            const auto                  iter = others_odom2map.find( other_name );
            if( iter != others_odom2map.end() ) {
                auto &odom_pose = others_odom_poses[other_name];
                // tf::poseMsgToEigen( pose_msg->pose, odom_pose.first );
                tf2::fromMsg( pose_msg->pose, odom_pose.first );
                update_pose( iter->second, odom_pose );

                other_position << odom_pose.second.position.x, odom_pose.second.position.y, odom_pose.second.position.z;

                pose_array_msg.poses.resize( others_odom_poses.size() );
                size_t i = 0;
                for( const auto &odom_pose : others_odom_poses ) {
                    pose_array_msg.poses[i].header     = pose_array_msg.header;
                    pose_array_msg.poses[i].robot_name = odom_pose.first;
                    pose_array_msg.poses[i].pose       = odom_pose.second.second;
                    i++;
                }
            }
        }

        // publish this information
        if( !pose_array_msg.poses.empty() ) {
            // others_poses_pub.publish( pose_array_msg );
            others_poses_pub->publish( pose_array_msg );
        }
    }


    /**
     * @brief receive anchor node pose from topic
     * @param
     */
    // void init_pose_callback( const nav_msgs::Odometry::ConstPtr &msg )
    void init_pose_callback( const nav_msgs::msg::Odometry::ConstSharedPtr msg )
    {
        std::lock_guard<std::mutex> lock( keyframe_queue_mutex );
        init_pose = msg;
    }

    /**
     * @brief Receive robot name from topic, used to request graph from other robots
     * @param msg Containing the robot name, which is used to request the graph from the robot with the same name
     */
    void request_robot_graph_callback( const std_msgs::msg::String::SharedPtr msg )
    {
        // call the publish graph function from within this class if the robot name is the own robot name
        if( msg->data == own_name ) {
            auto req = std::make_shared<const vamex_slam_msgs::srv::PublishGraph::Request>();
            auto res = std::make_shared<vamex_slam_msgs::srv::PublishGraph::Response>();
            publish_graph_service( req, res );
        }
    }

    /**
     * @brief update the cloud_msg with the latest map
     * @return true if cloud_msg was updated (cloud_msg can still be valid even if not updated)
     */
    bool update_cloud_msg()
    {
        if( !cloud_msg_update_required ) {
            return false;
        }

        std::vector<KeyFrameSnapshot::Ptr> snapshot;

        {
            std::lock_guard<std::mutex> lock( snapshots_mutex );

            snapshot                  = keyframes_snapshot;
            cloud_msg_update_required = false;
        }

        auto cloud = map_cloud_generator->generate( snapshot, map_cloud_resolution, map_cloud_count_threshold );
        if( !cloud ) {
            return false;
        }

        cloud->header.frame_id = map_frame_id;
        cloud->header.stamp    = snapshot.back()->cloud->header.stamp;

        if( !cloud_msg ) {
            // cloud_msg = sensor_msgs::PointCloud2Ptr( new sensor_msgs::PointCloud2() );
            cloud_msg = sensor_msgs::msg::PointCloud2::SharedPtr( new sensor_msgs::msg::PointCloud2() );
        }
        pcl::toROSMsg( *cloud, *cloud_msg );

        return true;
    }

    /**
     * @brief generate map point cloud and publish it
     * @param event
     */
    // void map_points_publish_timer_callback( const ros::WallTimerEvent &event )
    void map_points_publish_timer_callback()
    {
        // if( !map_points_pub.getNumSubscribers() ) {
        if( !map_points_pub->get_subscription_count() ) {
            return;
        }

        std::lock_guard<std::mutex> lock( cloud_msg_mutex );

        if( !update_cloud_msg() ) {
            return;
        }

        if( !cloud_msg ) {
            return;
        }

        // map_points_pub.publish( cloud_msg );
        map_points_pub->publish( *cloud_msg );
    }

    /**
     * @brief get the curren map as point cloud
     * @param req
     * @param res
     * @return
     */
    // bool get_map_service( hdl_graph_slam::GetMapRequest &req, hdl_graph_slam::GetMapResponse &res )
    // TODO test this service
    void get_map_service( vamex_slam_msgs::srv::GetMap::Request::ConstSharedPtr req, vamex_slam_msgs::srv::GetMap::Response::SharedPtr res )
    {
        std::lock_guard<std::mutex> lock( cloud_msg_mutex );

        update_cloud_msg();

        if( !cloud_msg ) {
            // ROS2 services are of type void and dont return a bool.
            // return false;
            return;
        }

        // == and != operators are defined for builtin_interfaces::msg::Time
        if( req->last_stamp != cloud_msg->header.stamp ) {
            // res.updated   = true;
            // res.cloud_map = *cloud_msg;
            res->updated   = true;
            res->cloud_map = *cloud_msg;
        } else {
            // res.updated = false;
            res->updated = false;
        }

        // ROS2 services are of type void and dont return a bool.
        // return true;
    }

    /**
     * @brief get the curren graph estimate
     * @param req
     * @param res
     * @return
     */
    // bool get_graph_estimate_service( hdl_graph_slam::GetGraphEstimateRequest &req, hdl_graph_slam::GetGraphEstimateResponse &res )
    void get_graph_estimate_service( vamex_slam_msgs::srv::GetGraphEstimate::Request::ConstSharedPtr req,
                                     vamex_slam_msgs::srv::GetGraphEstimate::Response::SharedPtr     res )
    {
        std::lock_guard<std::mutex> lock( graph_estimate_msg_mutex );

        if( graph_estimate_msg_update_required ) {
            if( keyframes_snapshot.empty() || edges_snapshot.empty() ) {
                // ROS2 services are of type void and dont return a bool.
                // return false;
                return;
            }

            if( !graph_estimate_msg ) {
                // graph_estimate_msg = hdl_graph_slam::GraphEstimatePtr( new hdl_graph_slam::GraphEstimate() );
                graph_estimate_msg = vamex_slam_msgs::msg::GraphEstimate::SharedPtr( new vamex_slam_msgs::msg::GraphEstimate() );
            }

            std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot_tmp;
            std::vector<EdgeSnapshot::Ptr>     edges_snapshot_tmp;

            {
                std::lock_guard<std::mutex> lock( snapshots_mutex );

                keyframes_snapshot_tmp = keyframes_snapshot;
                edges_snapshot_tmp     = edges_snapshot;
            }

            graph_estimate_msg->header.frame_id = map_frame_id;
            // pcl_conversions::fromPCL( keyframes_snapshot_tmp.back()->cloud->header.stamp, graph_estimate_msg->header.stamp );
            rclcpp::Time graph_estimate_stamp;
            pcl_conversions::fromPCL( keyframes_snapshot_tmp.back()->cloud->header.stamp, graph_estimate_stamp );
            graph_estimate_msg->header.stamp = graph_estimate_stamp.operator builtin_interfaces::msg::Time();

            graph_estimate_msg->edges.resize( edges_snapshot_tmp.size() );
            graph_estimate_msg->keyframes.resize( keyframes_snapshot_tmp.size() );

            for( size_t i = 0; i < edges_snapshot_tmp.size(); i++ ) {
                auto &edge_out    = graph_estimate_msg->edges[i];
                auto &edge_in     = edges_snapshot_tmp[i];
                edge_out.gid      = edge_in->gid;
                edge_out.from_gid = edge_in->from_gid;
                edge_out.to_gid   = edge_in->to_gid;
                edge_out.type     = static_cast<uint8_t>( edge_in->type );
            }

            for( size_t i = 0; i < keyframes_snapshot_tmp.size(); i++ ) {
                auto &keyframe_out = graph_estimate_msg->keyframes[i];
                auto &keyframe_in  = keyframes_snapshot_tmp[i];
                keyframe_out.gid   = keyframe_in->gid;
                keyframe_out.stamp = keyframe_in->stamp;
                // tf::poseEigenToMsg( keyframe_in->pose, keyframe_out.estimate.pose );
                keyframe_out.estimate.pose = tf2::toMsg( keyframe_in->pose );
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covMap( keyframe_out.estimate.covariance.data() );
                covMap = keyframe_in->covariance;
            }

            graph_estimate_msg_update_required = false;
        }

        if( !graph_estimate_msg ) {
            // ROS2 services are of type void and dont return a bool.
            // return false;
            return;
        }

        // if( req.last_stamp != graph_estimate_msg->header.stamp ) {
        //     res.updated        = true;
        //     res.graph_estimate = *graph_estimate_msg;
        // } else {
        //     res.updated = false;
        // }
        if( req->last_stamp != graph_estimate_msg->header.stamp ) {
            res->updated        = true;
            res->graph_estimate = *graph_estimate_msg;
        } else {
            res->updated = false;
        }

        // ROS2 services are of type void and dont return a bool.
        // return true;
    }

    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     * @param event
     */
    // void optimization_timer_callback( const ros::WallTimerEvent &event )
    void optimization_timer_callback()
    {
        std::lock_guard<std::mutex> lock( main_thread_mutex );

        // add keyframes and floor coeffs in the queues to the pose graph
        bool keyframe_updated = flush_keyframe_queue();

        if( !keyframe_updated ) {
            std_msgs::msg::Header read_until;
            // read_until.stamp    = ros::Time::now() + ros::Duration( 30, 0 );
            read_until.stamp    = ( this->now() + rclcpp::Duration( 30, 0 ) ).operator builtin_interfaces::msg::Time();
            read_until.frame_id = points_topic;
            // read_until_pub->publish( read_until );
            read_until_pub->publish( read_until );
            read_until.frame_id = "/filtered_points";
            // read_until_pub.publish( read_until );
            read_until_pub->publish( read_until );
        }

        if( !keyframe_updated & !flush_graph_queue()
            & !floor_coeffs_processor.flush( graph_slam, keyframes, keyframe_hash, prev_robot_keyframe->stamp )
            & !gps_processor.flush( graph_slam, keyframes ) & !imu_processor.flush( graph_slam, keyframes, base_frame_id ) ) {
            // Publish the current slam pose if nothing has been updated
            if( prev_robot_keyframe != nullptr ) {
                publish_slam_pose_broadcast( prev_robot_keyframe );
            }

            return;
        }

        // loop detection
        std::vector<Loop::Ptr> loops = loop_detector->detect( keyframes, new_keyframes, *graph_slam );
        for( const auto &loop : loops ) {
            Eigen::Isometry3d relpose( loop->relative_pose.cast<double>() );
            Eigen::MatrixXd   information_matrix = inf_calclator->calc_information_matrix( loop->key1->cloud, loop->key2->cloud, relpose );
            auto              graph_edge = graph_slam->add_se3_edge( loop->key1->node, loop->key2->node, relpose, information_matrix );
            auto              edge       = new Edge( graph_edge, Edge::TYPE_LOOP, loop->key1->gid, loop->key2->gid, *gid_generator );
            edges.emplace_back( edge );
            edge_gids.insert( edge->gid );
            graph_slam->add_robust_kernel( graph_edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size );
        }

        std::copy( new_keyframes.begin(), new_keyframes.end(), std::back_inserter( keyframes ) );
        new_keyframes.clear();

        // move the first node anchor position to the current estimate of the first node pose
        // so the first node moves freely while trying to stay around the origin
        // if( anchor_node && private_nh.param<bool>( "fix_first_node_adaptive", true ) ) {
        // For the multi robot case, fixing the first node adaptively (with initial positions that are not the identity transform) this
        // leads to ever moving pose graph
        if( anchor_node && fix_first_node_adaptive ) {
            Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3 *>( anchor_edge->vertices()[1] )->estimate();
            anchor_node->setEstimate( anchor_target );
        }

        if( keyframes.empty() ) {
            return;
        }

        // optimize the pose graph
        // int num_iterations = private_nh.param<int>( "g2o_solver_num_iterations", 1024 );
        graph_slam->optimize( g2o_solver_num_iterations );

        // get transformations between map and robots
        Eigen::Isometry3d               trans = prev_robot_keyframe->node->estimate() * prev_robot_keyframe->odom.inverse();
        std::vector<KeyFrame::ConstPtr> others_last_kf;
        trans_odom2map_mutex.lock();
        trans_odom2map = trans.matrix().cast<float>();
        others_last_kf.reserve( others_prev_robot_keyframes.size() );
        for( const auto &other_prev_kf : others_prev_robot_keyframes ) {
            //                                                node pointer                      odometry (not stored in kf for other
            //                                                robots)
            Eigen::Isometry3d other_trans        = other_prev_kf.second.first->node->estimate() * other_prev_kf.second.second.inverse();
            others_odom2map[other_prev_kf.first] = other_trans;
            // RCLCPP_INFO_STREAM( this->get_logger(), other_prev_kf.first );
            // RCLCPP_INFO_STREAM( this->get_logger(), "estimate:\n" << other_prev_kf.second.first->node->estimate().matrix() );
            // RCLCPP_INFO_STREAM( this->get_logger(), "odom:\n" << other_prev_kf.second.second.matrix() );
            // RCLCPP_INFO_STREAM( this->get_logger(), "trans:\n" << other_trans.matrix() );

            auto iter = others_odom_poses.find( other_prev_kf.first );

            if( iter != others_odom_poses.end() ) {
                update_pose( other_trans, iter->second );
                others_last_kf.emplace_back( other_prev_kf.second.first );
            }
        }
        trans_odom2map_mutex.unlock();

        auto marginals = graph_slam->compute_marginals();

        std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot_tmp( keyframes.size() );
        std::transform( keyframes.begin(), keyframes.end(), keyframes_snapshot_tmp.begin(),
                        [=]( const KeyFrame::Ptr &k ) { return std::make_shared<KeyFrameSnapshot>( k, marginals ); } );
        std::vector<EdgeSnapshot::Ptr> edges_snapshot_tmp( edges.size() );
        std::transform( edges.begin(), edges.end(), edges_snapshot_tmp.begin(),
                        [=]( const Edge::Ptr &e ) { return std::make_shared<EdgeSnapshot>( e ); } );

        snapshots_mutex.lock();
        keyframes_snapshot.swap( keyframes_snapshot_tmp );
        edges_snapshot.swap( edges_snapshot_tmp );
        cloud_msg_update_required          = true;
        graph_estimate_msg_update_required = true;
        snapshots_mutex.unlock();

        // Publish the current optimized slam pose
        publish_slam_pose_broadcast( prev_robot_keyframe );

        // if( odom2map_pub.getNumSubscribers() ) {
        if( odom2map_pub->get_subscription_count() ) {
            geometry_msgs::msg::TransformStamped ts = matrix2transform( prev_robot_keyframe->stamp, trans.matrix().cast<float>(),
                                                                        map_frame_id, odom_frame_id );
            // odom2map_pub.publish( ts );
            odom2map_pub->publish( ts );
        }

        if( markers_pub.getNumSubscribers() ) {
            markers_pub.publish( graph_slam, keyframes, edges, prev_robot_keyframe, others_last_kf, loop_detector->get_distance_thresh(),
                                 *gid_generator );
        }
        if( markers_pub.getNumMarginalsSubscribers() ) {
            markers_pub.publishMarginals( keyframes, marginals, *gid_generator );
        }
    }

    // /**
    //  * @brief dump all data to the current directory
    //  * @param req
    //  * @param res
    //  * @return
    //  */
    // bool dump_service( hdl_graph_slam::DumpGraphRequest &req, hdl_graph_slam::DumpGraphResponse &res )
    void dump_service( vamex_slam_msgs::srv::DumpGraph::Request::ConstSharedPtr req,
                       vamex_slam_msgs::srv::DumpGraph::Response::SharedPtr     res )
    {
        std::lock_guard<std::mutex> lock( main_thread_mutex );

        // std::string directory = req.destination;
        std::string directory = req->destination;

        if( directory.empty() ) {
            std::array<char, 64> buffer;
            buffer.fill( 0 );
            time_t rawtime;
            time( &rawtime );
            const auto timeinfo = localtime( &rawtime );
            strftime( buffer.data(), sizeof( buffer ), "%d-%m-%Y %H:%M:%S", timeinfo );
        }

        if( !boost::filesystem::is_directory( directory ) ) {
            boost::filesystem::create_directory( directory );
        }

        std::cout << "all data dumped to:" << directory << std::endl;

        graph_slam->save( directory + "/graph.g2o" );
        for( int i = 0; i < (int)keyframes.size(); i++ ) {
            std::stringstream sst;
            sst << boost::format( "%s/%06d" ) % directory % i;

            keyframes[i]->save( sst.str() );
        }

        const auto &zero_utm = gps_processor.zero_utm();
        if( zero_utm ) {
            std::ofstream zero_utm_ofs( directory + "/zero_utm" );
            zero_utm_ofs << boost::format( "%.6f %.6f %.6f" ) % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
        }

        std::ofstream ofs( directory + "/special_nodes.csv" );
        const auto   &floor_plane_node = floor_coeffs_processor.floor_plane_node();
        ofs << "anchor_node " << ( anchor_node == nullptr ? -1 : anchor_node->id() ) << std::endl;
        ofs << "anchor_edge " << ( anchor_edge == nullptr ? -1 : anchor_edge->id() ) << std::endl;
        ofs << "floor_node " << ( floor_plane_node == nullptr ? -1 : floor_plane_node->id() ) << std::endl;

        // res.success = true;
        res->success = true;
        // ROS2 services are of type void and dont return a bool.
        // return true;
    }

    /**
     * @brief save map data as pcd
     * @param req
     * @param res
     * @return
     */
    // bool save_map_service( hdl_graph_slam::SaveMapRequest &req, hdl_graph_slam::SaveMapResponse &res )
    void save_map_service( vamex_slam_msgs::srv::SaveMap::Request::ConstSharedPtr req,
                           vamex_slam_msgs::srv::SaveMap::Response::SharedPtr     res )
    {
        std::vector<KeyFrameSnapshot::Ptr> snapshot;

        snapshots_mutex.lock();
        snapshot = keyframes_snapshot;
        snapshots_mutex.unlock();

        // auto cloud = map_cloud_generator->generate( snapshot, req.resolution, req.count_threshold );
        auto cloud = map_cloud_generator->generate( snapshot, req->resolution, req->count_threshold );
        if( !cloud ) {
            // res.success = false;
            res->success = false;
            // ROS2 services are of type void and dont return a bool.
            // return true;
            return;
        }

        const auto &zero_utm = gps_processor.zero_utm();
        // if( zero_utm && req.utm ) {
        if( zero_utm && req->utm ) {
            for( auto &pt : cloud->points ) {
                pt.getVector3fMap() += ( *zero_utm ).cast<float>();
            }
        }

        cloud->header.frame_id = map_frame_id;
        cloud->header.stamp    = snapshot.back()->cloud->header.stamp;

        if( zero_utm ) {
            // std::ofstream ofs( req.destination + ".utm" );
            std::ofstream ofs( req->destination + ".utm" );
            ofs << boost::format( "%.6f %.6f %.6f" ) % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
        }

        // int ret     = pcl::io::savePCDFileBinary( req.destination, *cloud );
        // res.success = ret == 0;
        auto dir = boost::filesystem::path( req->destination ).remove_filename();
        if( !boost::filesystem::is_directory( dir ) ) {
            boost::filesystem::create_directory( dir );
        }
        int ret      = pcl::io::savePCDFileBinary( req->destination, *cloud );
        res->success = ret == 0;

        // ROS2 services are of type void and dont return a bool.
        // return true;
    }


    /**
     * @brief publish graph on corresponding topic
     * @param req
     * @param res
     * @return
     */
    // bool publish_graph_service( hdl_graph_slam::PublishGraphRequest &req, hdl_graph_slam::PublishGraphResponse &res )
    // TODO ROS2 test this
    void publish_graph_service( vamex_slam_msgs::srv::PublishGraph::Request::ConstSharedPtr req,
                                vamex_slam_msgs::srv::PublishGraph::Response::SharedPtr     res )
    {
        {
            std::lock_guard<std::mutex> lock( main_thread_mutex );

            if( keyframes.empty() ) {
                // ROS2 services are of type void and dont return a bool.
                // return false;
                return;
            }

            res->graph.robot_name          = own_name;
            res->graph.latest_keyframe_gid = prev_robot_keyframe->gid;
            // tf::poseEigenToMsg( prev_robot_keyframe->odom, res->graph.latest_keyframe_odom );
            res->graph.latest_keyframe_odom = tf2::toMsg( prev_robot_keyframe->odom );

            RCLCPP_DEBUG_STREAM( this->get_logger(), "Received publish graph request with the already processed keyframe gids: " );
            for( auto gid : req->processed_keyframe_gids ) {
                RCLCPP_DEBUG_STREAM( this->get_logger(), gid );
            }
            RCLCPP_DEBUG_STREAM( this->get_logger(), "Received publish graph request with the already processed edge gids: " );
            for( auto gid : req->processed_edge_gids ) {
                RCLCPP_DEBUG_STREAM( this->get_logger(), gid );
            }

            res->graph.keyframes.reserve( keyframes.size() );
            int added_keyframes = 0;
            for( size_t i = 0; i < keyframes.size(); i++ ) {
                auto &src = keyframes[i];
                // Skip adding keyframes that have already been processed by the other robot
                auto it_processed_gids = std::find( req->processed_keyframe_gids.begin(), req->processed_keyframe_gids.end(), src->gid );
                if( it_processed_gids != req->processed_keyframe_gids.end() ) {
                    RCLCPP_DEBUG_STREAM( this->get_logger(),
                                         "Skipping keyframe " << src->gid << " as it has already been processed by the other robot" );
                    continue;
                }

                // auto &dst = res->graph.keyframes[i];
                vamex_slam_msgs::msg::KeyFrameRos dst;
                dst.gid              = src->gid;
                dst.stamp            = src->stamp;
                dst.exclude_from_map = src->exclude_from_map;
                // tf::poseEigenToMsg( src->estimate(), dst.estimate );
                dst.estimate = tf2::toMsg( src->estimate() );
                dst.cloud    = *src->cloud_msg;
                res->graph.keyframes.push_back( std::move( dst ) );
                added_keyframes++;
            }
            res->graph.keyframes.resize( added_keyframes );

            res->graph.edges.reserve( edges.size() );
            int added_edges = 0;
            for( size_t i = 0; i < edges.size(); i++ ) {
                auto &src = edges[i];
                // Skip adding edges that have already been processed by the other robot
                auto it_processed_gids = std::find( req->processed_edge_gids.begin(), req->processed_edge_gids.end(), src->gid );
                if( it_processed_gids != req->processed_edge_gids.end() ) {
                    RCLCPP_DEBUG_STREAM( this->get_logger(),
                                         "Skipping edge " << src->gid << " as it has already been processed by the other robot" );
                    continue;
                }

                vamex_slam_msgs::msg::EdgeRos dst;
                dst.type     = src->type == Edge::TYPE_ODOM ? 0 : 1;
                dst.gid      = src->gid;
                dst.from_gid = src->from_gid;
                dst.to_gid   = src->to_gid;
                // tf::poseEigenToMsg( src->relative_pose(), dst.relative_pose );
                dst.relative_pose = tf2::toMsg( src->relative_pose() );
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> information_map( dst.information.data() );
                information_map = src->information();
                res->graph.edges.push_back( std::move( dst ) );
                added_edges++;
            }
            res->graph.edges.resize( added_edges );
        }

        // graph_broadcast_pub.publish( msg );
        RCLCPP_INFO_STREAM( this->get_logger(), "Sending graph response to " << req->robot_name << " with " << res->graph.keyframes.size()
                                                                             << " keyframes and " << res->graph.edges.size() << " edges." );
        RCLCPP_DEBUG_STREAM( this->get_logger(), "Response has the following keyframe gids: " );
        for( auto keyframe : res->graph.keyframes ) {
            RCLCPP_DEBUG_STREAM( this->get_logger(), keyframe.gid );
        }
        RCLCPP_DEBUG_STREAM( this->get_logger(), "Response has the following edge gids: " );
        for( auto edge : res->graph.edges ) {
            RCLCPP_DEBUG_STREAM( this->get_logger(), edge.gid );
        }
    }


    void request_graph_service( vamex_slam_msgs::srv::RequestGraphs::Request::ConstSharedPtr req,
                                vamex_slam_msgs::srv::RequestGraphs::Response::SharedPtr     res )
    {
        std::unique_lock<std::mutex> unique_lck( main_thread_mutex );

        vamex_slam_msgs::srv::PublishGraph::Request::SharedPtr pub_req = std::make_shared<vamex_slam_msgs::srv::PublishGraph::Request>();

        pub_req->robot_name = own_name;
        pub_req->processed_keyframe_gids.reserve( keyframes.size() );
        for( const auto &keyframe : keyframes ) {
            pub_req->processed_keyframe_gids.push_back( keyframe->gid );
        }
        pub_req->processed_edge_gids.reserve( edges.size() );
        for( const auto &edge : edges ) {
            pub_req->processed_edge_gids.push_back( edge->gid );
        }

        unique_lck.unlock();

        for( const auto &robot_name : req->robot_names ) {
            if( robot_name == own_name ) {
                continue;
            }
            auto it = std::find( robot_names.begin(), robot_names.end(), robot_name );
            if( it == robot_names.end() ) {
                RCLCPP_WARN_STREAM( this->get_logger(), "Robot " << robot_name << " is not in the list of known robots to request graph" );
                continue;
            }
            while( !request_graph_service_clients[robot_name]->wait_for_service( std::chrono::seconds( 2 ) ) ) {
                if( !rclcpp::ok() ) {
                    return;
                }
                RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), *this->get_clock(), 1000,
                                             "Waiting for service " << request_graph_service_clients[robot_name]->get_service_name()
                                                                    << " to appear..." );
            }

            auto               result_future = request_graph_service_clients[robot_name]->async_send_request( pub_req );
            std::future_status status        = result_future.wait_for( std::chrono::seconds( 1 ) );
            if( status == std::future_status::timeout ) {
                RCLCPP_ERROR_STREAM( this->get_logger(), "Request graph service call to rover " << robot_name << " timed out" );
                return;
            }
            if( status == std::future_status::ready ) {
                RCLCPP_INFO_STREAM( this->get_logger(), "Request graph service call to rover " << robot_name << " successful" );
            }
            auto result = result_future.get();
            // Fill the graph queue with the received graph
            std::lock_guard<std::mutex> lock( graph_queue_mutex );
            graph_queue.push_back( std::make_shared<vamex_slam_msgs::msg::GraphRos>( std::move( result->graph ) ) );

            others_last_accum_dist[robot_name] = others_slam_poses[robot_name].accum_dist;
        }
    }

    void get_graph_gids_service( vamex_slam_msgs::srv::GetGraphGids::Request::ConstSharedPtr req,
                                 vamex_slam_msgs::srv::GetGraphGids::Response::SharedPtr     res )
    {
        std::lock_guard<std::mutex> lock( main_thread_mutex );

        res->keyframe_gids.reserve( keyframes.size() );
        for( const auto &keyframe : keyframes ) {
            res->keyframe_gids.push_back( keyframe->gid );
        }
        res->edge_gids.reserve( edges.size() );
        for( const auto &edge : edges ) {
            res->edge_gids.push_back( edge->gid );
        }
    }

private:
    // ROS
    // ros::NodeHandle nh;
    // ros::NodeHandle mt_nh;
    // ros::NodeHandle private_nh;
    // ros::WallTimer optimization_timer;
    // ros::WallTimer map_publish_timer;
    rclcpp::TimerBase::SharedPtr one_shot_initalization_timer;
    rclcpp::TimerBase::SharedPtr optimization_timer;
    rclcpp::TimerBase::SharedPtr map_publish_timer;

    // std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>>       odom_sub;
    // std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
    // std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>>       sync;
    std::string                                                      odom_sub_topic;
    message_filters::Subscriber<nav_msgs::msg::Odometry>             odom_sub;
    std::string                                                      cloud_sub_topic;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2>       cloud_sub;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

    // ros::Subscriber graph_broadcast_sub;
    // ros::Publisher  graph_broadcast_pub;
    rclcpp::Publisher<vamex_slam_msgs::msg::PoseWithName>::SharedPtr                               slam_pose_broadcast_pub;
    std::unordered_map<std::string, rclcpp::Client<vamex_slam_msgs::srv::PublishGraph>::SharedPtr> request_graph_service_clients;
    std::unordered_map<std::string, vamex_slam_msgs::msg::PoseWithName>                            others_slam_poses;
    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr                            slam_pose_broadcast_sub;
    // other robot name -> accumulated dist when last graph update was requested from that robot
    std::unordered_map<std::string, double> others_last_accum_dist;
    double                                  graph_request_min_accum_dist;
    double                                  graph_request_max_robot_dist;

    // ros::Subscriber odom_broadcast_sub;
    // ros::Publisher  odom_broadcast_pub;
    // ros::Publisher  others_poses_pub;
    rclcpp::Subscription<vamex_slam_msgs::msg::PoseWithName>::SharedPtr   odom_broadcast_sub;
    rclcpp::Publisher<vamex_slam_msgs::msg::PoseWithName>::SharedPtr      odom_broadcast_pub;
    rclcpp::Publisher<vamex_slam_msgs::msg::PoseWithNameArray>::SharedPtr others_poses_pub;

    std::mutex      trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map;
    // ros::Publisher                                     odom2map_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;
    std::unordered_map<std::string, Eigen::Isometry3d>                 others_odom2map;  // odom2map transform for other robots
    // std::unordered_map<std::string, std::pair<Eigen::Isometry3d, geometry_msgs::Pose>> others_odom_poses;
    std::unordered_map<std::string, std::pair<Eigen::Isometry3d, geometry_msgs::msg::Pose>> others_odom_poses;


    // ros::Publisher read_until_pub;
    // ros::Publisher map_points_pub;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr         read_until_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;

    // std::unordered_map<std::string, ros::ServiceClient> request_graph_service_clients;
    // ros::ServiceServer                                  dump_service_server;
    // ros::ServiceServer                                  save_map_service_server;
    // ros::ServiceServer                                  get_map_service_server;
    // ros::ServiceServer                                  get_graph_estimate_service_server;
    // ros::ServiceServer                                  publish_graph_service_server;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             request_robot_graph_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                request_robot_graph_pub;
    rclcpp::Service<vamex_slam_msgs::srv::DumpGraph>::SharedPtr        dump_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::SaveMap>::SharedPtr          save_map_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::GetMap>::SharedPtr           get_map_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::GetGraphEstimate>::SharedPtr get_graph_estimate_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::PublishGraph>::SharedPtr     publish_graph_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::RequestGraphs>::SharedPtr    request_graph_service_server;
    rclcpp::Service<vamex_slam_msgs::srv::GetGraphGids>::SharedPtr     get_graph_gids_service_server;

    ImuProcessor         imu_processor;
    GpsProcessor         gps_processor;
    FloorCoeffsProcessor floor_coeffs_processor;

    MarkersPublisher markers_pub;

    // latest point cloud map
    std::mutex       cloud_msg_mutex;
    std::atomic_bool cloud_msg_update_required;
    // sensor_msgs::PointCloud2Ptr cloud_msg;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;

    // latest graph estimate
    std::mutex       graph_estimate_msg_mutex;
    std::atomic_bool graph_estimate_msg_update_required;
    // hdl_graph_slam::GraphEstimatePtr graph_estimate_msg;
    vamex_slam_msgs::msg::GraphEstimate::SharedPtr graph_estimate_msg;

    // getting init pose from topic
    // ros::Subscriber init_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr init_pose_sub;
    std::string                                              init_pose_topic;
    // nav_msgs::Odometry::ConstPtr init_pose;  // should be accessed with keyframe_queue_mutex locked
    nav_msgs::msg::Odometry::ConstSharedPtr init_pose;  // should be accessed with keyframe_queue_mutex locked

    // keyframe queue
    std::string               base_frame_id;
    std::mutex                keyframe_queue_mutex;
    std::deque<KeyFrame::Ptr> keyframe_queue;

    // graph queue
    std::mutex graph_queue_mutex;
    // std::deque<hdl_graph_slam::GraphRosConstPtr> graph_queue;
    std::deque<vamex_slam_msgs::msg::GraphRos::ConstSharedPtr> graph_queue;

    // for map cloud generation and graph publishing
    std::string                        map_frame_id;
    std::string                        odom_frame_id;
    double                             map_cloud_resolution;
    double                             map_cloud_count_threshold;
    std::mutex                         snapshots_mutex;
    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
    std::vector<EdgeSnapshot::Ptr>     edges_snapshot;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;

    // global id
    std::unique_ptr<GlobalIdGenerator> gid_generator;

    // More parameters
    std::vector<rclcpp::Parameter> param_vec;  // Externally provided by manual composition (debugging)

    std::string              points_topic;
    std::string              own_name;
    std::vector<std::string> robot_names;
    std::string              model_namespace;

    float               robot_remove_points_radius;
    std::vector<double> init_pose_vec;
    bool                fix_first_node;
    bool                fix_first_node_adaptive;
    std::vector<double> fix_first_node_stddev_vec;
    std::string         odometry_edge_robust_kernel;
    double              odometry_edge_robust_kernel_size;
    std::string         loop_closure_edge_robust_kernel;
    double              loop_closure_edge_robust_kernel_size;
    std::string         g2o_solver_type;
    bool                save_graph;
    int                 g2o_solver_num_iterations;
    double              graph_update_interval;
    double              map_cloud_update_interval;


    // graph slam
    // all the below members must be accessed after locking main_thread_mutex
    std::mutex main_thread_mutex;

    int                       max_keyframes_per_update;
    std::deque<KeyFrame::Ptr> new_keyframes;
    KeyFrame::ConstPtr        prev_robot_keyframe;

    std::unordered_map<std::string, std::pair<KeyFrame::ConstPtr, Eigen::Isometry3d>> others_prev_robot_keyframes;

    g2o::VertexSE3            *anchor_node;
    g2o::EdgeSE3              *anchor_edge;
    std::vector<KeyFrame::Ptr> keyframes;
    // std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;
    // TODO clarify whether builtin_interfaces::msg::Time or rclcpp::Time should be used
    std::unordered_map<builtin_interfaces::msg::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;
    std::vector<Edge::Ptr>                                                        edges;
    std::unordered_map<GlobalId, KeyFrame::Ptr>                                   keyframe_gids;
    std::unordered_set<GlobalId>                                                  edge_gids;
    std::unordered_set<GlobalId>                                                  edge_ignore_gids;

    std::shared_ptr<GraphSLAM>       graph_slam;
    std::unique_ptr<LoopDetector>    loop_detector;
    std::unique_ptr<KeyframeUpdater> keyframe_updater;

    std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}  // namespace hdl_graph_slam

// PLUGINLIB_EXPORT_CLASS( hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet )
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hdl_graph_slam::HdlGraphSlamComponent )