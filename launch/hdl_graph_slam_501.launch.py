import os
import yaml

from ament_index_python import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory("hdl_graph_slam"), 
        "config",
        "params.yaml"
    )

    with open(config_file_path, "r") as file:
        config_params = yaml.safe_load(file)
        print(yaml.dump( config_params, default_flow_style=False))
        prefiltering_config_params =  config_params["/prefiltering_component"]["ros__parameters"]

    # Create the container node
    container = Node(
        name='hdl_graph_slam_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    # https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html
    load_composable_nodes = LoadComposableNodes(
        target_container="hdl_graph_slam_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdl_graph_slam",
                plugin="hdl_graph_slam::PrefilteringComponent",
                name="prefiltering_component",
                parameters=[prefiltering_config_params],
                extra_arguments=[{"use_intra_process_comms": True}]  # TODO verify
            )
        ]
    )
    
    return LaunchDescription([container, load_composable_nodes])