import os
import yaml

from ament_index_python import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        static_transform_params = config_params["/lidar2base_publisher"]["ros__parameters"]
        prefiltering_params = config_params["/prefiltering_component"]["ros__parameters"]
        scan_matching_odometry_params = config_params["/scan_matching_odometry_component"]["ros__parameters"]
        floor_detection_params = config_params["/floor_detection_component"]["ros__parameters"]
        hdl_grapp_slam_params = config_params["/hdl_graph_slam_component"]["ros__parameters"]

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
        output="screen"
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
        remappings=[("/velodyne_points", shared_params["points_topic"])],  # TODO verify how often remapping is needed
        extra_arguments=[{"use_intra_process_comms": False}]  # TODO verify
    )

    scan_matching_odometry_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::ScanMatchingOdometryComponent",
        name="scan_matching_odometry_component",
        parameters=[scan_matching_odometry_params, shared_params],
        # remappings=[("/filtered_points", "/prefiltering/filtered_points")],
        extra_arguments=[{"use_intra_process_comms": False}]  # TODO verify
    )

    floor_detection_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::FloorDetectionComponent",
        name="floor_detection_component",
        parameters=[floor_detection_params, shared_params],
        remappings=[("/filtered_points", "/prefiltering/filtered_points")],
        extra_arguments=[{"use_intra_process_comms": False}]  # TODO verify
    )

    hdl_graph_slam_node = ComposableNode(
        package="hdl_graph_slam",
        plugin="hdl_graph_slam::HdlGraphSlamComponent",
        name="hdl_graph_slam_component",
        parameters=[hdl_grapp_slam_params, shared_params],
        remappings=[
            ("/filtered_points", "/prefiltering/filtered_points"),
            ("odom", "/scan_matching_odometry/odom"),
            ("floor_coeffs", "/floor_detection/floor_coeffs")
        ],
        extra_arguments=[{"use_intra_process_comms": False}]  # TODO verify
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

    return LaunchDescription([static_transform_publisher, container, load_composable_nodes])
