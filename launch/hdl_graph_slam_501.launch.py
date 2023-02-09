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
        "params.yaml"
    )

    with open(config_file_path, "r") as file:
        config_params = yaml.safe_load(file)
        # Print all parameters from the yaml file for convenience when launching the nodes
        print(yaml.dump( config_params, default_flow_style=False))
        shared_params = config_params["/**"]["ros__parameters"]
        prefiltering_config_params =  config_params["/prefiltering_component"]["ros__parameters"]

    # Create the container node
    container = Node(
        name='hdl_graph_slam_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        parameters=[shared_params]
    )

    # https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html
    load_composable_nodes = LoadComposableNodes(
        target_container="hdl_graph_slam_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdl_graph_slam",
                plugin="hdl_graph_slam::PrefilteringComponent",
                name="prefiltering_component",
                parameters=[prefiltering_config_params, shared_params],
                extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
            )
        ]
    )
    
    return LaunchDescription([container, load_composable_nodes])