
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

// ROS2
#include <angles/angles.h>

namespace mrg_slam {

class MrgRelayComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;

    MrgRelayComponent( const rclcpp::NodeOptions& options ) : Node( "mrg_relay_component", options )
    {
        // Initialize the relay component
        RCLCPP_INFO( this->get_logger(), "MrgRelayComponent initialized" );
    }

private:
    void initialize_params()
    {
        // General and scenario parameters
        points_topic      = declare_parameter<std::string>( "points_topic", "velodyne_points" );
        own_name          = declare_parameter<std::string>( "own_name", "atlas" );
        multi_robot_names = declare_parameter<std::vector<std::string>>( "multi_robot_names", { "atlas", "bestla" } );
        odom_sub_topic    = declare_parameter<std::string>( "odom_sub_topic", "/scan_matching_odometry/odom" );
        cloud_sub_topic   = declare_parameter<std::string>( "cloud_sub_topic", "/prefiltering/filtered_points" );

        // Initial pose parameters
        init_odom_topic = declare_parameter<std::string>( "init_odom_topic", "NONE" );
        init_pose_topic = declare_parameter<std::string>( "init_pose_topic", "NONE" );
        init_pose_vec   = declare_parameter<std::vector<double>>( "init_pose", std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } );


        // GraphSLAM parameters
        fix_first_node_adaptive   = declare_parameter<bool>( "fix_first_node_adaptive", false );
        g2o_solver_type           = declare_parameter<std::string>( "g2o_solver_type", "lm_var_cholmod" );
        g2o_solver_num_iterations = declare_parameter<int>( "g2o_solver_num_iterations", 1024 );
        save_graph                = declare_parameter<bool>( "save_graph", true );
        declare_parameter<bool>( "g2o_verbose", false );
        graph_update_interval               = declare_parameter<double>( "graph_update_interval", 3.0 );
        map_cloud_update_interval           = declare_parameter<double>( "map_cloud_update_interval", 10.0 );
        graph_request_min_accum_dist        = declare_parameter<double>( "graph_request_min_accum_dist", 3.0 );
        graph_request_max_robot_dist        = declare_parameter<double>( "graph_request_max_robot_dist", 10.0 );
        graph_request_min_time_delay        = declare_parameter<double>( "graph_request_min_time_delay", 5.0 );
        std::string graph_exchange_mode_str = declare_parameter<std::string>( "graph_exchange_mode", "PATH_PROXIMITY" );
        graph_exchange_mode                 = graph_exchange_mode_from_string( graph_exchange_mode_str );


        // GraphDatabase parameters (not directly used by this class)
        declare_parameter<bool>( "fix_first_node", false );
        declare_parameter<std::vector<double>>( "fix_first_node_stddev",
                                                std::vector<double>{ 0.5, 0.5, 0.5, angles::from_degrees( 5 ), angles::from_degrees( 5 ),
                                                                     angles::from_degrees( 5 ) } );
        declare_parameter<int>( "max_keyframes_per_update", 10 );
        declare_parameter<std::string>( "odometry_edge_robust_kernel", "NONE" );
        declare_parameter<double>( "odometry_edge_robust_kernel_size", 1.0 );
        declare_parameter<std::string>( "loop_closure_edge_robust_kernel", "Huber" );
        declare_parameter<double>( "loop_closure_edge_robust_kernel_size", 1.0 );
        declare_parameter<std::string>( "result_dir", "" );

        // LoopDetector parameters (not directly used by this class)
        declare_parameter<double>( "distance_thresh", 5.0 );
        declare_parameter<double>( "accum_distance_thresh", 8.0 );
        declare_parameter<double>( "accum_distance_thresh_other_slam_instance", 7.5 );
        declare_parameter<double>( "fitness_score_max_range", std::numeric_limits<double>::max() );
        declare_parameter<double>( "fitness_score_thresh", 0.5 );
        declare_parameter<bool>( "use_planar_registration_guess", false );
        declare_parameter<bool>( "use_loop_closure_consistency_check", true );
        declare_parameter<double>( "loop_closure_consistency_max_delta_trans", 0.25 );
        declare_parameter<double>( "loop_closure_consistency_max_delta_angle", 5 );
    }


    enum GraphExchangeMode {
        CURRENT_PROXIMITY,
        PATH_PROXIMITY,
    };

    GraphExchangeMode graph_exchange_mode_from_string( const std::string& str )
    {
        // Transform to upper case
        if( str == "CURRENT_PROXIMITY" ) {
            return GraphExchangeMode::CURRENT_PROXIMITY;
        } else if( str == "PATH_PROXIMITY" ) {
            return GraphExchangeMode::PATH_PROXIMITY;
        } else {
            throw std::runtime_error( "Unknown graph exchange mode: " + str );
        }
    }

    // timers
    rclcpp::TimerBase::SharedPtr optimization_timer;


    // More parameters
    std::string              points_topic;
    std::string              own_name;
    std::vector<std::string> multi_robot_names;
};

}  // namespace mrg_slam
