// SPDX-License-Identifier: BSD-2-Clause

#include <angles/angles.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <hdl_graph_slam/msg/graph_ros.hpp>
#include <hdl_graph_slam/msg/pose_with_name.hpp>
#include <hdl_graph_slam/msg/pose_with_name_array.hpp>
#include <hdl_graph_slam/srv/dump_graph.hpp>
#include <hdl_graph_slam/srv/get_graph_estimate.hpp>
#include <hdl_graph_slam/srv/get_map.hpp>
#include <hdl_graph_slam/srv/publish_graph.hpp>
#include <hdl_graph_slam/srv/save_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
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
    HdlGraphSlamComponent( const rclcpp::NodeOptions &options ) : Node( "hdl_graph_slam_component", options )
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
        RCLCPP_INFO( this->get_logger(), "Initializing hdl_graph_slam nodelet..." );
        // Deactivate this timer immediately so the initialization is only performed once
        one_shot_initalization_timer->cancel();

        // Get the shared pointer to the ROS2 node to pass it to other classes and functions
        auto node_ros = shared_from_this();
        // This class is the node handle as it is derived from rclcpp::Node
        // nh         = getNodeHandle();
        // mt_nh      = getMTNodeHandle();
        // private_nh = getPrivateNodeHandle();

        // init parameters
        // map_frame_id              = private_nh.param<std::string>( "map_frame_id", "map" );
        // odom_frame_id             = private_nh.param<std::string>( "odom_frame_id", "odom" );
        // map_cloud_resolution      = private_nh.param<double>( "map_cloud_resolution", 0.05 );
        // map_cloud_count_threshold = private_nh.param<int>( "map_cloud_count_threshold", 2 );
        // Declare only once across all nodes
        map_frame_id              = this->declare_parameter<std::string>( "map_frame_id", "map" );
        odom_frame_id             = this->declare_parameter<std::string>( "odom_frame_id", "odom" );
        map_cloud_resolution      = this->declare_parameter<double>( "map_cloud_resolution", 0.05 );
        map_cloud_count_threshold = this->declare_parameter<int>( "map_cloud_count_threshold", 2 );
        trans_odom2map.setIdentity();

        // max_keyframes_per_update = private_nh.param<int>( "max_keyframes_per_update", 10 );
        max_keyframes_per_update = this->declare_parameter<int>( "max_keyframes_per_update", 10 );

        robot_remove_points_radius = static_cast<float>( this->declare_parameter<double>( "robot_remove_points_radius", 2.0 ) );
        init_pose_vec  = this->declare_parameter<std::vector<double>>( "init_pose", std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } );
        fix_first_node = this->declare_parameter<bool>( "fix_first_node", false );
        fix_first_node_adaptive   = this->declare_parameter<bool>( "fix_first_node_adaptive", false );
        fix_first_node_stddev_vec = this->declare_parameter<std::vector<double>>(
            "fix_first_node_stddev",
            std::vector<double>{ 0.5, 0.5, 0.5, angles::from_degrees( 5 ), angles::from_degrees( 5 ), angles::from_degrees( 5 ) } );
        odometry_edge_robust_kernel          = this->declare_parameter<std::string>( "odometry_edge_robust_kernel", "NONE" );
        odometry_edge_robust_kernel_size     = this->declare_parameter<double>( "odometry_edge_robust_kernel_size", 1.0 );
        loop_closure_edge_robust_kernel      = this->declare_parameter<std::string>( "loop_closure_edge_robust_kernel", "Huber" );
        loop_closure_edge_robust_kernel_size = this->declare_parameter<double>( "loop_closure_edge_robust_kernel_size", 1.0 );
        graph_request_min_accum_dist         = this->declare_parameter<double>( "graph_request_min_accum_dist", 2.0 );
        graph_request_max_robot_dist         = this->declare_parameter<double>( "graph_request_max_robot_dist", 10.0 );
        g2o_solver_num_iterations            = this->declare_parameter<int>( "g2o_solver_num_iterations", 1024 );

        //
        anchor_node = nullptr;
        anchor_edge = nullptr;
        // graph_slam.reset( new GraphSLAM( private_nh.param<std::string>( "g2o_solver_type", "lm_var" ) ) );
        graph_slam.reset( new GraphSLAM( this->declare_parameter<std::string>( "g2o_solver_type", "lm_var_cholmod" ) ) );
        graph_slam->set_save_graph( this->declare_parameter<bool>( "save_graph", true ) );
        // keyframe_updater.reset( new KeyframeUpdater( private_nh ) );
        // loop_detector.reset( new LoopDetector( private_nh ) );
        // map_cloud_generator.reset( new MapCloudGenerator() );
        // inf_calclator.reset( new InformationMatrixCalculator( private_nh ) );

        keyframe_updater.reset( new KeyframeUpdater( node_ros ) );
        loop_detector.reset( new LoopDetector( node_ros ) );
        map_cloud_generator.reset( new MapCloudGenerator() );
        inf_calclator.reset( new InformationMatrixCalculator( node_ros ) );

        // points_topic = private_nh.param<std::string>( "points_topic", "/velodyne_points" );
        points_topic = this->declare_parameter<std::string>( "points_topic", "/velodyne_points" );

        // own_name = private_nh.param<std::string>( "own_name", "atlas" );
        // Declare only once across nodes
        own_name = this->has_parameter( "own_name" ) ? this->get_parameter( "own_name" ).as_string()
                                                     : this->declare_parameter<std::string>( "own_name", "atlas" );
        // robot_names.push_back( own_name );
        // robot_names   = private_nh.param<std::vector<std::string>>( "/properties/scenario/rovers/names", robot_names );
        robot_names = this->declare_parameter<std::vector<std::string>>( "/properties/scenario/rovers/names", robot_names );
        // gid_generator = std::unique_ptr<GlobalIdGenerator>( new GlobalIdGenerator( own_name, robot_names ) );
        gid_generator = std::unique_ptr<GlobalIdGenerator>( new GlobalIdGenerator( node_ros, own_name, robot_names ) );

        // subscribers
        // odom_sub.reset( new message_filters::Subscriber<nav_msgs::Odometry>( mt_nh, "/odom", 256 ) );
        // cloud_sub.reset( new message_filters::Subscriber<sensor_msgs::PointCloud2>( mt_nh, "/filtered_points", 32 ) );
        // TODO further specify qos profile
        std::string model_namespace = this->declare_parameter<std::string>( "model_namespace", "" );
        std::string odom_sub_topic  = this->declare_parameter<std::string>( "odom_sub_topic", "/odom" );
        std::string cloud_sub_topic = this->declare_parameter<std::string>( "cloud_sub_topic", "/filtered_points" );
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
        // graph_broadcast_sub = nh.subscribe( "/hdl_graph_slam/graph_broadcast", 16, &HdlGraphSlamComponent::graph_callback, this );
        // odom_broadcast_sub  = nh.subscribe( "/hdl_graph_slam/odom_broadcast", 16, &HdlGraphSlamComponent::odom_broadcast_callback, this
        // );
        graph_broadcast_sub = this->create_subscription<hdl_graph_slam::msg::GraphRos>( "/hdl_graph_slam/graph_broadcast",
                                                                                        rclcpp::QoS( 16 ),
                                                                                        std::bind( &HdlGraphSlamComponent::graph_callback,
                                                                                                   this, std::placeholders::_1 ) );
        odom_broadcast_sub  = this->create_subscription<hdl_graph_slam::msg::PoseWithName>(
            "/hdl_graph_slam/odom_broadcast", rclcpp::QoS( 16 ),
            std::bind( &HdlGraphSlamComponent::odom_broadcast_callback, this, std::placeholders::_1 ) );

        // init_pose_topic = private_nh.param<std::string>( "init_pose_topic", "NONE" );
        init_pose_topic = this->declare_parameter<std::string>( "init_pose_topic", "NONE" );
        if( init_pose_topic != "NONE" ) {
            init_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>( init_pose_topic, rclcpp::QoS( 32 ),
                                                                                std::bind( &HdlGraphSlamComponent::init_pose_callback, this,
                                                                                           std::placeholders::_1 ) );
        }

        // publishers
        // odom2map_pub        = mt_nh.advertise<geometry_msgs::TransformStamped>( "/hdl_graph_slam/odom2pub", 16 );
        // map_points_pub      = mt_nh.advertise<sensor_msgs::PointCloud2>( "/hdl_graph_slam/map_points", 1, true );
        // read_until_pub      = mt_nh.advertise<std_msgs::Header>( "/hdl_graph_slam/read_until", 16 );
        // graph_broadcast_pub = mt_nh.advertise<hdl_graph_slam::GraphRos>( "/hdl_graph_slam/graph_broadcast", 16 );
        // odom_broadcast_pub  = mt_nh.advertise<hdl_graph_slam::PoseWithName>( "/hdl_graph_slam/odom_broadcast", 16 );
        // others_poses_pub    = mt_nh.advertise<hdl_graph_slam::PoseWithNameArray>( "/hdl_graph_slam/others_poses", 16 );
        odom2map_pub        = this->create_publisher<geometry_msgs::msg::TransformStamped>( "/hdl_graph_slam/odom2pub", 16 );
        map_points_pub      = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/hdl_graph_slam/map_points", rclcpp::QoS( 1 ) );
        read_until_pub      = this->create_publisher<std_msgs::msg::Header>( "/hdl_graph_slam/read_until", rclcpp::QoS( 16 ) );
        graph_broadcast_pub = this->create_publisher<hdl_graph_slam::msg::GraphRos>( "/hdl_graph_slam/graph_broadcast", rclcpp::QoS( 16 ) );
        odom_broadcast_pub  = this->create_publisher<hdl_graph_slam::msg::PoseWithName>( "/hdl_graph_slam/odom_broadcast",
                                                                                        rclcpp::QoS( 16 ) );
        others_poses_pub    = this->create_publisher<hdl_graph_slam::msg::PoseWithNameArray>( "/hdl_graph_slam/others_poses",
                                                                                           rclcpp::QoS( 16 ) );

        // dump_service_server     = mt_nh.advertiseService( "/hdl_graph_slam/dump", &HdlGraphSlamComponent::dump_service, this );
        // save_map_service_server = mt_nh.advertiseService( "/hdl_graph_slam/save_map", &HdlGraphSlamComponent::save_map_service, this );
        // get_map_service_server  = mt_nh.advertiseService( "/hdl_graph_slam/get_map", &HdlGraphSlamComponent::get_map_service, this );
        // get_graph_estimate_service_server = mt_nh.advertiseService( "/hdl_graph_slam/get_graph_estimate",
        //                                                             &HdlGraphSlamComponent::get_graph_estimate_service, this );
        // publish_graph_service_server      = mt_nh.advertiseService( "/hdl_graph_slam/publish_graph",
        //                                                             &HdlGraphSlamComponent::publish_graph_service, this );
        // We need to define a special function to pass arguments to a ROS2 callback with multiple parameters when the callback is a class
        // member function, see https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
        // If these service callbacks dont work during runtime, try using lambda functions as in
        // https://github.com/ros2/demos/blob/foxy/demo_nodes_cpp/src/services/add_two_ints_server.cpp
        // Dumb service
        std::function<void( const std::shared_ptr<hdl_graph_slam::srv::DumpGraph::Request> req,
                            std::shared_ptr<hdl_graph_slam::srv::DumpGraph::Response>      res )>
            dump_service_callback = std::bind( &HdlGraphSlamComponent::dump_service, this, std::placeholders::_1, std::placeholders::_2 );
        dump_service_server       = this->create_service<hdl_graph_slam::srv::DumpGraph>( "/hdl_graph_slam/dump", dump_service_callback );
        // Save map service
        std::function<void( const std::shared_ptr<hdl_graph_slam::srv::SaveMap::Request> req,
                            std::shared_ptr<hdl_graph_slam::srv::SaveMap::Response>      res )>
            save_map_service_callback = std::bind( &HdlGraphSlamComponent::save_map_service, this, std::placeholders::_1,
                                                   std::placeholders::_2 );
        save_map_service_server       = this->create_service<hdl_graph_slam::srv::SaveMap>( "/hdl_graph_slam/save_map",
                                                                                      save_map_service_callback );
        // Get map service
        std::function<void( const std::shared_ptr<hdl_graph_slam::srv::GetMap::Request> req,
                            std::shared_ptr<hdl_graph_slam::srv::GetMap::Response>      res )>
            get_map_service_callback = std::bind( &HdlGraphSlamComponent::get_map_service, this, std::placeholders::_1,
                                                  std::placeholders::_2 );
        get_map_service_server = this->create_service<hdl_graph_slam::srv::GetMap>( "/hdl_graph_slam/get_map", get_map_service_callback );
        // Get graph estimate service
        std::function<void( const std::shared_ptr<hdl_graph_slam::srv::GetGraphEstimate::Request> req,
                            std::shared_ptr<hdl_graph_slam::srv::GetGraphEstimate::Response>      res )>
            get_graph_estimate_service_callback = std::bind( &HdlGraphSlamComponent::get_graph_estimate_service, this,
                                                             std::placeholders::_1, std::placeholders::_2 );
        get_graph_estimate_service_server       = this->create_service<hdl_graph_slam::srv::GetGraphEstimate>(
            "/hdl_graph_slam/get_graph_estimate", get_graph_estimate_service_callback );
        // Publish graph service
        std::function<void( const std::shared_ptr<hdl_graph_slam::srv::PublishGraph::Request> req,
                            std::shared_ptr<hdl_graph_slam::srv::PublishGraph::Response>      res )>
            publish_graph_service_callback = std::bind( &HdlGraphSlamComponent::publish_graph_service, this, std::placeholders::_1,
                                                        std::placeholders::_2 );
        publish_graph_service_server       = this->create_service<hdl_graph_slam::srv::PublishGraph>( "/hdl_graph_slam/publish_graph",
                                                                                                publish_graph_service_callback );

        for( const auto &robot_name : robot_names ) {
            if( robot_name != own_name ) {
                // request_graph_service_clients[robot_name] = mt_nh.serviceClient<hdl_graph_slam::PublishGraph>(
                // "/" + robot_name + "/hdl_graph_slam/publish_graph" );
                // The service client from within this component is not working, so we create a publisher and subscriber to request the due
                // to adding the same node to the exectuor multiple times
                // graph request_graph_service_clients[robot_name] = this->create_client<hdl_graph_slam::srv::PublishGraph>(
                //     "/" + robot_name + "/hdl_graph_slam/publish_graph" );
                others_accum_dist[robot_name] = std::make_pair<double, double>( 0, -1 );
            }
        }
        // Instead of creating a client for each robot, we subscribe to a topic that requests the graph from other robots
        request_robot_graph_sub = this->create_subscription<std_msgs::msg::String>(
            "/hdl_graph_slam/request_robot_graph", rclcpp::QoS( 16 ),
            std::bind( &HdlGraphSlamComponent::request_robot_graph_callback, this, std::placeholders::_1 ) );
        request_robot_graph_pub = this->create_publisher<std_msgs::msg::String>( "/hdl_graph_slam/request_robot_graph", rclcpp::QoS( 16 ) );

        cloud_msg_update_required          = false;
        graph_estimate_msg_update_required = false;
        // double graph_update_interval       = private_nh.param<double>( "graph_update_interval", 3.0 );
        // double map_cloud_update_interval   = private_nh.param<double>( "map_cloud_update_interval", 10.0 );
        double graph_update_interval     = this->declare_parameter<double>( "graph_update_interval", 3.0 );
        double map_cloud_update_interval = this->declare_parameter<double>( "map_cloud_update_interval", 10.0 );

        // optimization_timer                 = mt_nh.createWallTimer( ros::WallDuration( graph_update_interval ),
        //                                                             &HdlGraphSlamNodelet::optimization_timer_callback, this );
        // map_publish_timer                  = mt_nh.createWallTimer( ros::WallDuration( map_cloud_update_interval ),
        //                                                             &HdlGraphSlamNodelet::map_points_publish_timer_callback, this );
        optimization_timer = rclcpp::create_timer( node_ros, node_ros->get_clock(), rclcpp::Duration( graph_update_interval, 0 ),
                                                   std::bind( &HdlGraphSlamComponent::optimization_timer_callback, this ) );
        map_publish_timer  = rclcpp::create_timer( node_ros, node_ros->get_clock(), rclcpp::Duration( map_cloud_update_interval, 0 ),
                                                   std::bind( &HdlGraphSlamComponent::map_points_publish_timer_callback, this ) );

        // gps_processor.onInit( nh, mt_nh, private_nh );
        // imu_processor.onInit( nh, mt_nh, private_nh );
        // floor_coeffs_processor.onInit( nh, mt_nh, private_nh );
        // markers_pub.onInit( nh, mt_nh, private_nh );
        gps_processor.onInit( node_ros );
        imu_processor.onInit( node_ros );
        floor_coeffs_processor.onInit( node_ros );
        markers_pub.onInit( node_ros );

        // Optionally print the all parameters declared in this node so far
        const auto &list_params = this->list_parameters( std::vector<std::string>{}, 0 );
        print_ros2_parameters( this->get_parameters( list_params.names ), this->get_logger() );
    }

private:
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
        hdl_graph_slam::msg::PoseWithName pose_msg;
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
            keyframe_gids[keyframe->gid] = keyframe;
            // TODO clarify whether builtin_interfaces::msg::Time or rclcpp::Time should be used, it seems like it should be
            // builtin_interfaces::msg::Time
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
    void graph_callback( hdl_graph_slam::msg::GraphRos::ConstSharedPtr graph_msg )
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
        std::unordered_map<GlobalId, const hdl_graph_slam::msg::KeyFrameRos *> unique_keyframes;
        std::unordered_map<GlobalId, const hdl_graph_slam::msg::EdgeRos *>     unique_edges;

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
        RCLCPP_INFO_STREAM( this->get_logger(), "Unique edges:     " << unique_edges.size() );
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

            if( from_keyframe->edge_exists( *to_keyframe ) ) {
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


    /**
     * @brief received odom msgs from other robots proccessed
     * @param graph_msg
     */
    // void odom_broadcast_callback( const hdl_graph_slam::PoseWithName::ConstPtr &pose_msg )
    void odom_broadcast_callback( hdl_graph_slam::msg::PoseWithName::ConstSharedPtr pose_msg )
    {
        if( pose_msg->robot_name == own_name ) {
            return;
        }

        // PoseWithNameArray pose_array_msg;
        hdl_graph_slam::msg::PoseWithNameArray pose_array_msg;
        pose_array_msg.header.stamp    = pose_msg->header.stamp;
        pose_array_msg.header.frame_id = map_frame_id;

        Eigen::Vector3d other_position, own_position;
        std::string     other_name           = pose_msg->robot_name;
        bool            other_position_valid = false;

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
                other_position_valid = true;

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

        // check whether the graph should be requested of this rover
        {
            std::lock_guard<std::mutex> lock( main_thread_mutex );
            if( !prev_robot_keyframe ) {
                return;
            }
            own_position = prev_robot_keyframe->node->estimate().translation();
        }

        auto   &accum_dist_pair = others_accum_dist[other_name];
        double  accum_dist      = accum_dist_pair.first;   // get a copy of accum_dist to allow for updating the pair with current value
        double &last_accum_dist = accum_dist_pair.second;  // get a reference here, because it may be updated below
        accum_dist_pair.first   = pose_msg->accum_dist;
        // double min_accum_dist   = private_nh.param<double>( "graph_request_min_accum_dist", 2 );
        if( last_accum_dist >= 0 && fabs( last_accum_dist - accum_dist ) < graph_request_min_accum_dist ) {
            return;
        }

        // double max_robot_dist_sqr = private_nh.param<double>( "graph_request_max_robot_dist", 10 );
        double max_robot_dist_sqr = graph_request_max_robot_dist * graph_request_max_robot_dist;
        if( last_accum_dist >= 0 && ( !other_position_valid || ( own_position - other_position ).squaredNorm() > max_robot_dist_sqr ) ) {
            return;
        }

        // if( request_graph_service_clients[other_name].call( srv ) ) {
        //     ROS_INFO_STREAM( "Successfully requested graph from " << other_name );
        //     last_accum_dist = accum_dist;
        // } else {
        //     ROS_WARN_STREAM( "Failed to request graph from " << other_name );
        // }
        // Check if service call succeeds for ROS2 humble or rolling can be performed as in
        // https://github.com/ros2/examples/blob/rolling/rclcpp/services/minimal_client/main.cpp
        // However for ROS2 foxy client->remove_pending_request() is not available.
        // Thats why we rely on the foxy implementation and hope it works
        // https://github.com/ros2/examples/blob/foxy/rclcpp/services/minimal_client/main.cpp
        // TODO Service client call needs to be done from a different thread/node, multi robot not working
        // auto client        = request_graph_service_clients[other_name];
        // auto request       = std::make_shared<hdl_graph_slam::srv::PublishGraph::Request>();
        // auto result_future = client->async_send_request( request );
        // if( rclcpp::spin_until_future_complete( shared_from_this(), result_future ) != rclcpp::FutureReturnCode::SUCCESS ) {
        //     RCLCPP_ERROR_STREAM( this->get_logger(), "Failed to request graph from " << other_name );
        //     client->remove_pending_request( result_future );
        // } else {
        //     RCLCPP_INFO_STREAM( this->get_logger(), "Successfully requested graph from " << other_name );
        //     last_accum_dist = accum_dist;
        // }
        // Instead we just publish a message to the request a specific robot graph
        std_msgs::msg::String msg;
        msg.data = other_name;
        request_robot_graph_pub->publish( msg );
        // TODO only update last_accum_dist if the request was successful
        last_accum_dist = accum_dist;
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
            auto req = std::make_shared<const hdl_graph_slam::srv::PublishGraph::Request>();
            auto res = std::make_shared<hdl_graph_slam::srv::PublishGraph::Response>();
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
    void get_map_service( hdl_graph_slam::srv::GetMap::Request::ConstSharedPtr req, hdl_graph_slam::srv::GetMap::Response::SharedPtr res )
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
    void get_graph_estimate_service( hdl_graph_slam::srv::GetGraphEstimate::Request::ConstSharedPtr req,
                                     hdl_graph_slam::srv::GetGraphEstimate::Response::SharedPtr     res )
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
                graph_estimate_msg = hdl_graph_slam::msg::GraphEstimate::SharedPtr( new hdl_graph_slam::msg::GraphEstimate() );
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
            // ROS_INFO_STREAM( other_prev_kf.first );
            // ROS_INFO_STREAM( "estimate:\n" << other_prev_kf.second.first->node->estimate().matrix() );
            // ROS_INFO_STREAM( "odom:\n" << other_prev_kf.second.second.matrix() );
            // ROS_INFO_STREAM( "trans:\n" << other_trans.matrix() );

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
    void dump_service( hdl_graph_slam::srv::DumpGraph::Request::ConstSharedPtr req,
                       hdl_graph_slam::srv::DumpGraph::Response::SharedPtr     res )
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
    void save_map_service( hdl_graph_slam::srv::SaveMap::Request::ConstSharedPtr req,
                           hdl_graph_slam::srv::SaveMap::Response::SharedPtr     res )
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
    void publish_graph_service( hdl_graph_slam::srv::PublishGraph::Request::ConstSharedPtr req,
                                hdl_graph_slam::srv::PublishGraph::Response::SharedPtr     res )
    {
        // GraphRos msg;
        hdl_graph_slam::msg::GraphRos msg;
        {
            std::lock_guard<std::mutex> lock( main_thread_mutex );

            if( keyframes.empty() ) {
                // ROS2 services are of type void and dont return a bool.
                // return false;
                return;
            }

            msg.robot_name          = own_name;
            msg.latest_keyframe_gid = prev_robot_keyframe->gid;
            // tf::poseEigenToMsg( prev_robot_keyframe->odom, msg.latest_keyframe_odom );
            msg.latest_keyframe_odom = tf2::toMsg( prev_robot_keyframe->odom );

            msg.keyframes.resize( keyframes.size() );
            for( size_t i = 0; i < keyframes.size(); i++ ) {
                auto &dst            = msg.keyframes[i];
                auto &src            = keyframes[i];
                dst.gid              = src->gid;
                dst.stamp            = src->stamp;
                dst.exclude_from_map = src->exclude_from_map;
                // tf::poseEigenToMsg( src->estimate(), dst.estimate );
                dst.estimate = tf2::toMsg( src->estimate() );
                dst.cloud    = *src->cloud_msg;
            }

            msg.edges.resize( edges.size() );
            for( size_t i = 0; i < edges.size(); i++ ) {
                auto &dst    = msg.edges[i];
                auto &src    = edges[i];
                dst.type     = src->type == Edge::TYPE_ODOM ? 0 : 1;
                dst.gid      = src->gid;
                dst.from_gid = src->from_gid;
                dst.to_gid   = src->to_gid;
                // tf::poseEigenToMsg( src->relative_pose(), dst.relative_pose );
                dst.relative_pose = tf2::toMsg( src->relative_pose() );
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> information_map( dst.information.data() );
                information_map = src->information();
            }
        }

        // graph_broadcast_pub.publish( msg );
        graph_broadcast_pub->publish( msg );

        // ROS2 services are of type void and dont return a bool.
        // return true;
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
    message_filters::Subscriber<nav_msgs::msg::Odometry>             odom_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2>       cloud_sub;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

    // ros::Subscriber graph_broadcast_sub;
    // ros::Publisher  graph_broadcast_pub;
    rclcpp::Subscription<hdl_graph_slam::msg::GraphRos>::SharedPtr graph_broadcast_sub;
    rclcpp::Publisher<hdl_graph_slam::msg::GraphRos>::SharedPtr    graph_broadcast_pub;

    std::string map_frame_id;
    std::string odom_frame_id;

    // ros::Subscriber odom_broadcast_sub;
    // ros::Publisher  odom_broadcast_pub;
    // ros::Publisher  others_poses_pub;
    rclcpp::Subscription<hdl_graph_slam::msg::PoseWithName>::SharedPtr   odom_broadcast_sub;
    rclcpp::Publisher<hdl_graph_slam::msg::PoseWithName>::SharedPtr      odom_broadcast_pub;
    rclcpp::Publisher<hdl_graph_slam::msg::PoseWithNameArray>::SharedPtr others_poses_pub;

    std::mutex      trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map;
    // ros::Publisher                                     odom2map_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;
    std::unordered_map<std::string, Eigen::Isometry3d>                 others_odom2map;  // odom2map transform for other robots
    // std::unordered_map<std::string, std::pair<Eigen::Isometry3d, geometry_msgs::Pose>> others_odom_poses;
    std::unordered_map<std::string, std::pair<Eigen::Isometry3d, geometry_msgs::msg::Pose>> others_odom_poses;
    std::unordered_map<std::string, std::pair<double, double>> others_accum_dist;  // first:  accumulated dist of other robot,
                                                                                   // second: accumulated dist when last graph update
                                                                                   // was requested from that robot

    std::string points_topic;
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
    // std::unordered_map<std::string, rclcpp::Client<hdl_graph_slam::srv::PublishGraph>::SharedPtr> request_graph_service_clients;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            request_robot_graph_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               request_robot_graph_pub;
    rclcpp::Service<hdl_graph_slam::srv::DumpGraph>::SharedPtr        dump_service_server;
    rclcpp::Service<hdl_graph_slam::srv::SaveMap>::SharedPtr          save_map_service_server;
    rclcpp::Service<hdl_graph_slam::srv::GetMap>::SharedPtr           get_map_service_server;
    rclcpp::Service<hdl_graph_slam::srv::GetGraphEstimate>::SharedPtr get_graph_estimate_service_server;
    rclcpp::Service<hdl_graph_slam::srv::PublishGraph>::SharedPtr     publish_graph_service_server;

    std::string              own_name;
    std::vector<std::string> robot_names;

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
    hdl_graph_slam::msg::GraphEstimate::SharedPtr graph_estimate_msg;

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
    std::deque<hdl_graph_slam::msg::GraphRos::ConstSharedPtr> graph_queue;

    // for map cloud generation and graph publishing
    double                             map_cloud_resolution;
    double                             map_cloud_count_threshold;
    std::mutex                         snapshots_mutex;
    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
    std::vector<EdgeSnapshot::Ptr>     edges_snapshot;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;

    // global id
    std::unique_ptr<GlobalIdGenerator> gid_generator;

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

    // More parameters
    float               robot_remove_points_radius;
    std::vector<double> init_pose_vec;
    bool                fix_first_node;
    bool                fix_first_node_adaptive;
    std::vector<double> fix_first_node_stddev_vec;
    std::string         odometry_edge_robust_kernel;
    double              odometry_edge_robust_kernel_size;
    std::string         loop_closure_edge_robust_kernel;
    double              loop_closure_edge_robust_kernel_size;
    double              graph_request_min_accum_dist;
    double              graph_request_max_robot_dist;
    int                 g2o_solver_num_iterations;
};

}  // namespace hdl_graph_slam

// PLUGINLIB_EXPORT_CLASS( hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet )
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hdl_graph_slam::HdlGraphSlamComponent )