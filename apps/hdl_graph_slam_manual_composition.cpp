#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "floor_detection_component.cpp"
// #include "hdl_graph_slam_component.cpp"
#include "prefiltering_component.cpp"
#include "scan_matching_odometry_component.cpp"

// This file is needed to debug the hdl_graph_slam composition of nodes

int
main( int argc, char const *argv[] )
{
    // Force flush of the stdout buffer
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );

    // Initialize any global resources needed by the middleware and the client library.
    rclcpp::init( argc, argv );

    // Create an executor that will be used compose components
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions                       options;

    // Add our nodes to the executor
    auto prefiltering_component = std::make_shared<hdl_graph_slam::PrefilteringComponent>( options );
    exec.add_node( prefiltering_component );
    auto floor_detection_component = std::make_shared<hdl_graph_slam::FloorDetectionComponent>( options );
    exec.add_node( floor_detection_component );
    auto scan_matching_odometry_component = std::make_shared<hdl_graph_slam::ScanMatchingOdometryComponent>( options );
    exec.add_node( scan_matching_odometry_component );
    // auto hdl_graph_slam_component = std::make_shared<hdl_graph_slam::HdlGraphSlamComponent>( options );
    // exec.add_node( hdl_graph_slam_component );

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
