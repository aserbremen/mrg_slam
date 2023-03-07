import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory("hdl_graph_slam"),
        "config",
        "hdl_graph_slam_501.yaml"
    )

    with open(config_file_path, "r") as file:
        config_params = yaml.safe_load(file)
        # Print all parameters from the yaml file for convenience when launching the nodes
        print(yaml.dump(config_params, sort_keys=False, default_flow_style=False))
        shared_params = config_params["/**"]["ros__parameters"]
        static_transform_params = config_params["lidar2base_publisher"]["ros__parameters"]
        clock_publisher_ros2_params = config_params["clock_publisher_ros2"]["ros__parameters"]
        prefiltering_params = config_params["prefiltering_component"]["ros__parameters"]
        scan_matching_odometry_params = config_params["scan_matching_odometry_component"]["ros__parameters"]
        floor_detection_params = config_params["floor_detection_component"]["ros__parameters"]
        hdl_graph_slam_params = config_params["hdl_graph_slam_component"]["ros__parameters"]

    # Create the static transform publisher node
    static_transform_publisher = Node(
        name="lidar2base_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments has to be a list of strings
        arguments=[str(static_transform_params["x"]),
                   str(static_transform_params["y"]),
                   str(static_transform_params["z"]),
                   str(static_transform_params["roll"]),
                   str(static_transform_params["pitch"]),
                   str(static_transform_params["yaw"]),
                   str(static_transform_params["base_frame_id"]),
                   str(static_transform_params["lidar_frame_id"])],
        parameters=[shared_params],
        output="both"
    )

    # Start rviz2 from this launch file if set to true
    if shared_params["start_rviz2"]:
        print(os.path.join(get_package_share_directory("hdl_graph_slam"), "rviz", "hdl_graph_slam_ros2.rviz")),
        rviz2 = Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(get_package_share_directory("hdl_graph_slam"), "rviz", "hdl_graph_slam_ros2.rviz")],
            parameters=[shared_params],
            output="both"
        )

    # In case we play a rosbag in ROS2 foxy, we need to publish the clock from the rosbag to the /clock topic
    if os.path.expandvars("$ROS_DISTRO") != "humble":
        clock_publisher_ros2 = Node(
            name="clock_publisher_ros2",
            package="hdl_graph_slam",
            executable="clock_publisher_ros2.py",
            parameters=[clock_publisher_ros2_params, shared_params],
            output="both"
        )

    # Create the map2odom publisher node
    map2odom_publisher_ros2 = Node(
        name="map2odom_publisher_ros2",
        package="hdl_graph_slam",
        executable="map2odom_publisher_ros2.py",
        output="both",
        parameters=[shared_params]
    )

    # Create the container node
    container = Node(
        name="hdl_graph_slam_container",
        package="rclcpp_components",
        executable="component_container",
        output="both",
        parameters=[shared_params]
    )

    # Create the composable nodes
    prefiltering_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::PrefilteringComponent",
        name="prefiltering_component",
        parameters=[prefiltering_params, shared_params],
        # TODO verify how often remapping is needed
        remappings=[("/velodyne_points", shared_params["points_topic"])],
        extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
    )

    scan_matching_odometry_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::ScanMatchingOdometryComponent",
        name="scan_matching_odometry_component",
        parameters=[scan_matching_odometry_params, shared_params],
        # remappings=[("/filtered_points", "/prefiltering/filtered_points")],
        extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
    )

    floor_detection_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::FloorDetectionComponent",
        name="floor_detection_component",
        parameters=[floor_detection_params, shared_params],
        remappings=[("/filtered_points", "/prefiltering/filtered_points")],
        extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
    )

    hdl_graph_slam_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::HdlGraphSlamComponent",
        name="hdl_graph_slam_component",
        parameters=[hdl_graph_slam_params, shared_params],
        remappings=[
            ("/filtered_points", "/prefiltering/filtered_points"),
            ("/odom", "/scan_matching_odometry/odom"),
            ("/floor_coeffs", "/floor_detection/floor_coeffs")
        ],
        extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
    )

    composable_nodes = [prefiltering_node, scan_matching_odometry_node]
    if floor_detection_params["enable_floor_detection"]:
        composable_nodes.append(floor_detection_node)
    composable_nodes.append(hdl_graph_slam_node)

    # https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html
    load_composable_nodes = LoadComposableNodes(
        target_container="hdl_graph_slam_container",
        composable_node_descriptions=composable_nodes
    )

    # Create a list of the nodes to be launched
    launch_description_list = [static_transform_publisher]
    if [shared_params["start_rviz2"]]:
        launch_description_list.append(rviz2)
    # For ROS2 foxy we need to add our own clock publisher, from ROS2 humble we can publish the clock topic with ros2 bag play <bag> --clock
    if os.path.expandvars("$ROS_DISTRO") != "humble":
        launch_description_list.append(clock_publisher_ros2)
    launch_description_list.append(map2odom_publisher_ros2)
    launch_description_list.append(container)
    launch_description_list.append(load_composable_nodes)

    # Finally launch all nodes
    return LaunchDescription(launch_description_list)
