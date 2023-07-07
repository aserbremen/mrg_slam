#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "floor_detection_component.cpp"
#include "hdl_graph_slam_component.cpp"
#include "prefiltering_component.cpp"
#include "scan_matching_odometry_component.cpp"

// This executable is primarily used for debugging and development purposes. Compile using:
// colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_MANUAL_COMPOSITION=ON
// In terminal: ros2 run --prefix 'gdbserver localhost:3000' hdl_graph_slam manual_composition
// Debug using the launch.json file in the .vscode folder (see https://gist.github.com/JADC362/a4425c2d05cdaadaaa71b697b674425f)
// Parameter loading via yaml is unclear. If certain paramaters need to be set, specify the default values in the respective component
// For full functionality you should run the map2odom_publish node before debugging this composition:
// ros2 run hdl_graph_slam map2odom_publisher_ros2.py

int
main( int argc, char const *argv[] )
{
    if( argc != 2 ) {
        std::cout << "Usage: ros2 run hdl_graph_slam hdl_graph_slam_manual_composition <path_to_config_yaml>" << std::endl;
        return 1;
    }

    // Force flush of the stdout buffer
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );

    // Initialize any global resources needed by the middleware and the client library.
    rclcpp::init( argc, argv );

    std::string config_path = argv[1];
    std::cout << "Trying to parse config: " << config_path << std::endl;
    rclcpp::ParameterMap param_map_direct = rclcpp::parameter_map_from_yaml_file( config_path );
    for( auto const &node_params : param_map_direct ) {
        std::cout << "All parameters for node: " << node_params.first << std::endl;
        for( auto const &param : node_params.second ) {
            std::cout << std::left << std::setw( 28 ) << std::setfill( ' ' ) << param.get_name() << " " << param.value_to_string()
                      << std::endl;
        }
    }

    // Create an executor that will be used compose components
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions                       options;
    options.use_intra_process_comms( true );

    // Add our nodes to the executor
    auto prefiltering_component = std::make_shared<hdl_graph_slam::PrefilteringComponent>( options );
    exec.add_node( prefiltering_component );
    auto floor_detection_component = std::make_shared<hdl_graph_slam::FloorDetectionComponent>( options );
    exec.add_node( floor_detection_component );
    auto scan_matching_odometry_component = std::make_shared<hdl_graph_slam::ScanMatchingOdometryComponent>( options );
    exec.add_node( scan_matching_odometry_component );
    auto hdl_graph_slam_component = std::make_shared<hdl_graph_slam::HdlGraphSlamComponent>( options );
    exec.add_node( hdl_graph_slam_component );

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
