import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

# Parameter type mapping to infer the correct data type from the cli string
PARAM_MAPPING = {
    'model_namespace': str,
    'use_sim_time': bool,
    'start_rviz2': bool,
    'enable_floor_detection': bool,
    'enable_gps': bool,
    'enable_imu_acceleration': bool,
    'enable_imu_orientation': bool,
    'tf_link_values': str,
    'points_topic': str,
    'map_frame_id': str,
    'odom_frame_id': str,
    'robot_odom_frame_id': str,
    'enable_robot_odometry_init_guess': bool,
    'imu_topic': str,
    'x': float,
    'y': float,
    'z': float,
    'roll': float,
    'pitch': float,
    'yaw': float,
    'init_pose_topic': str,
    'enable_multi_robot_communicator': bool,
}


# Overwrite the parameters from the yaml file with the ones from the cli if the cli string is not empty
def overwrite_yaml_params_from_cli(yaml_params, cli_params):
    for key, value in cli_params.items():
        if key in yaml_params and value != '':
            # Since all parameters from cli in ROS2 are strings, we need to infer the correct data type
            yaml_params[key] = PARAM_MAPPING[key](value)
            # Overwrite the boolean values since they are not correctly parsed, non empty strings are always True
            if value == 'true' or value == 'True':
                yaml_params[key] = True
            elif value == 'false' or value == 'False':
                yaml_params[key] = False
    return yaml_params


# Print unsorted yaml parameters
def print_yaml_params(yaml_params, header=None):
    if header is not None:
        print('######## ' + header + ' ########')
    print(yaml.dump(yaml_params, sort_keys=False, default_flow_style=False))


def launch_setup(context, *args, **kwargs):

    config_file_path = os.path.join(
        get_package_share_directory('hdl_graph_slam'),
        'config',
        'hdl_multi_robot_graph_slam.yaml'
    )
    if 'config' in context.launch_configurations:
        config_file_path = context.launch_configurations['config']

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)
        print('Loaded config file: ' + config_file_path)
        # # Print all parameters from the yaml file for convenience when launching the nodes
        # print(yaml.dump(config_params, sort_keys=False, default_flow_style=False))
        shared_params = config_params['/**']['ros__parameters']
        static_transform_params = config_params['lidar2base_publisher']['ros__parameters']
        clock_publisher_ros2_params = config_params['clock_publisher_ros2']['ros__parameters']
        prefiltering_params = config_params['prefiltering_component']['ros__parameters']
        scan_matching_odometry_params = config_params['scan_matching_odometry_component']['ros__parameters']
        floor_detection_params = config_params['floor_detection_component']['ros__parameters']
        hdl_graph_slam_params = config_params['hdl_graph_slam_component']['ros__parameters']
        multi_robot_communicator_params = config_params['multi_robot_communicator']['ros__parameters']

    # Overwrite the parameters from the yaml file with the ones from the cli
    shared_params = overwrite_yaml_params_from_cli(shared_params, context.launch_configurations)
    static_transform_params = overwrite_yaml_params_from_cli(static_transform_params, context.launch_configurations)
    prefiltering_params = overwrite_yaml_params_from_cli(prefiltering_params, context.launch_configurations)
    scan_matching_odometry_params = overwrite_yaml_params_from_cli(scan_matching_odometry_params, context.launch_configurations)
    floor_detection_params = overwrite_yaml_params_from_cli(floor_detection_params, context.launch_configurations)
    hdl_graph_slam_params = overwrite_yaml_params_from_cli(hdl_graph_slam_params, context.launch_configurations)
    multi_robot_communicator_params = overwrite_yaml_params_from_cli(multi_robot_communicator_params, context.launch_configurations)

    model_namespace = shared_params['model_namespace']

    print_yaml_params(shared_params, 'shared_params')
    print_yaml_params(static_transform_params, 'static_transform_params')
    print_yaml_params(clock_publisher_ros2_params, 'clock_publisher_ros2_params')
    print_yaml_params(prefiltering_params, 'prefiltering_params')
    print_yaml_params(scan_matching_odometry_params, 'scan_matching_odometry_params')
    print_yaml_params(floor_detection_params, 'floor_detection_params')
    print_yaml_params(hdl_graph_slam_params, 'hdl_graph_slam_params')
    print_yaml_params(multi_robot_communicator_params, 'multi_robot_communicator_params')

    # Create the static transform publisher node
    frame_id = model_namespace + '/' + static_transform_params['base_frame_id']
    child_frame_id = model_namespace + '/' + static_transform_params['lidar_frame_id']
    static_transform_publisher = Node(
        # name=model_namespace + '_lidar2base_publisher',
        name='lidar2base_publisher',
        namespace=model_namespace,
        package='tf2_ros',
        executable='static_transform_publisher',
        # arguments has to be a list of strings
        arguments=[str(static_transform_params['lidar2base_x']),
                   str(static_transform_params['lidar2base_y']),
                   str(static_transform_params['lidar2base_z']),
                   str(static_transform_params['lidar2base_roll']),
                   str(static_transform_params['lidar2base_pitch']),
                   str(static_transform_params['lidar2base_yaw']),
                   frame_id,
                   child_frame_id],
        parameters=[shared_params],
        output='both'
    )

    # Start rviz2 from this launch file if set to true
    if shared_params['start_rviz2']:
        rviz2 = Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(
                'hdl_graph_slam'), 'rviz', 'hdl_multi_robot_graph_slam_ros2.rviz')],
            parameters=[shared_params],
            output='both'
        )

    # In case we play a rosbag in ROS2 foxy, we need to publish the clock from the rosbag to the /clock topic
    if os.path.expandvars('$ROS_DISTRO') != 'humble':
        clock_publisher_ros2 = Node(
            package='hdl_graph_slam',
            executable='clock_publisher_ros2.py',
            name=model_namespace + '_clock_publisher_ros2',
            parameters=[clock_publisher_ros2_params, shared_params],
            output='both'
        )

    # Create the map2odom publisher node
    map2odom_publisher_ros2 = Node(
        package='hdl_graph_slam',
        executable='map2odom_publisher_ros2.py',
        name='map2odom_publisher_ros2',
        namespace=model_namespace,
        output='both',
        parameters=[hdl_graph_slam_params, shared_params],
        remappings=[('/hdl_graph_slam/odom2pub', '/' + model_namespace + '/hdl_graph_slam/odom2pub')]
    )

    # Create the container node
    # container_name = model_namespace + '_hdl_graph_slam_container'
    # If the launch command provides the debug argument we add the prefix to start gdbserver (move this to the node you need to debug)
    # More information can be found in hdl_multi_robot_graph_slam_debug.launch.py and at https://gist.github.com/JADC362/a4425c2d05cdaadaaa71b697b674425f
    if 'debug' in context.launch_configurations:
        prefix = ['gdbserver localhost:3000']
    else:
        prefix = []
    container_name = model_namespace + '/hdl_graph_slam_container'  # used in composable nodes
    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name="hdl_graph_slam_container",
        namespace=model_namespace,
        output='both',
        parameters=[shared_params],
        prefix=prefix
    )

    # Create the composable nodes, change names, topics, remappings to avoid conflicts for the multi robot case
    prefiltering_params['base_link_frame'] = model_namespace + '/' + prefiltering_params['base_link_frame']
    if prefiltering_params['enable_prefiltering']:
        prefiltering_node = ComposableNode(
            package='hdl_graph_slam',
            plugin='hdl_graph_slam::PrefilteringComponent',
            name='prefiltering_component',
            namespace=model_namespace,
            parameters=[prefiltering_params, shared_params],
            remappings=[
                ('/imu/data', shared_params['imu_topic']),
                ('/velodyne_points', '/' + model_namespace + shared_params['points_topic']),
                ('/prefiltering/filtered_points', '/' + model_namespace + '/prefiltering/filtered_points'),
                ('/prefiltering/colored_points', '/' + model_namespace + '/prefiltering/colored_points'),
                # (prefiltering_params['keyed_scan_topic'], '/' + model_namespace + prefiltering_params['keyed_scan_topic']),
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    if scan_matching_odometry_params['enable_scan_matching_odometry']:
        scan_matching_odometry_node = ComposableNode(
            package='hdl_graph_slam',
            plugin='hdl_graph_slam::ScanMatchingOdometryComponent',
            name='scan_matching_odometry_component',
            namespace=model_namespace,
            parameters=[scan_matching_odometry_params, shared_params],
            remappings=[
                ('/points_topic', '/' + model_namespace + shared_params['points_topic']),
                ('/filtered_points', '/' + model_namespace + '/prefiltering/filtered_points'),
                ('/scan_matching_odometry/transform', '/' + model_namespace + '/scan_matching_odometry/transform'),
                ('/scan_matching_odometry/read_until', '/' + model_namespace + '/scan_matching_odometry/read_until'),
                ('/scan_matching_odometry/status', '/' + model_namespace + '/scan_matching_odometry/status'),
                ('/scan_matching_odometry/odom', '/' + model_namespace + '/scan_matching_odometry/odom'),
                ('/scan_matching_odometry/aligned_points', '/' + model_namespace + '/scan_matching_odometry/aligned_points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # TODO somehwere simple "/floor_coeffs" topic is either subscribed or published to even with namespace and remappings, but not sure where
    if floor_detection_params['enable_floor_detection']:
        floor_detection_node = ComposableNode(
            package='hdl_graph_slam',
            plugin='hdl_graph_slam::FloorDetectionComponent',
            name='floor_detection_component',
            namespace=model_namespace,
            parameters=[floor_detection_params, shared_params],
            remappings=[
                ('/points_topic', '/' + model_namespace + shared_params['points_topic']),
                ('/filtered_points', '/' + model_namespace + '/prefiltering/filtered_points'),
                ('/floor_detection/floor_coeffs', '/' + model_namespace + '/floor_detection/floor_coeffs'),
                ('/floor_detection/floor_filtered_points', '/' + model_namespace + '/floor_detection/floor_filtered_points'),
                ('/floor_detection/read_until', '/' + model_namespace + '/floor_detection/read_until'),
                ('/floor_detection/floor_points', '/' + model_namespace + '/floor_detection/floor_points')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    hdl_graph_slam_params['own_name'] = model_namespace
    # Overwrite init_pose array with the actual values
    hdl_graph_slam_params['init_pose'][0] = hdl_graph_slam_params['x']
    hdl_graph_slam_params['init_pose'][1] = hdl_graph_slam_params['y']
    hdl_graph_slam_params['init_pose'][2] = hdl_graph_slam_params['z']
    hdl_graph_slam_params['init_pose'][3] = hdl_graph_slam_params['yaw']
    hdl_graph_slam_params['init_pose'][4] = hdl_graph_slam_params['pitch']
    hdl_graph_slam_params['init_pose'][5] = hdl_graph_slam_params['roll']
    hdl_graph_slam_node = ComposableNode(
        package='hdl_graph_slam',
        plugin='hdl_graph_slam::HdlGraphSlamComponent',
        name='hdl_graph_slam_component',
        namespace=model_namespace,
        parameters=[hdl_graph_slam_params, shared_params],
        remappings=[
            ('/imu/data', shared_params['imu_topic']),
            ('/filtered_points', '/' + model_namespace + '/prefiltering/filtered_points'),
            ('/odom', '/' + model_namespace + '/scan_matching_odometry/odom'),
            ('floor_coeffs', '/' + model_namespace + '/floor_detection/floor_coeffs'),
            ('/hdl_graph_slam/map_points', '/' + model_namespace + '/hdl_graph_slam/map_points'),
            ('/hdl_graph_slam/markers', '/' + model_namespace + '/hdl_graph_slam/markers'),
            ('/hdl_graph_slam/markers_covariance', '/' + model_namespace + '/hdl_graph_slam/markers_covariance'),
            ('/hdl_graph_slam/odom2pub', '/' + model_namespace + '/hdl_graph_slam/odom2pub'),
            ('/hdl_graph_slam/read_until', '/' + model_namespace + '/hdl_graph_slam/read_until'),
            ('/hdl_graph_slam/others_poses', '/' + model_namespace + '/hdl_graph_slam/others_poses'),
            ('/hdl_graph_slam/publish_graph', '/' + model_namespace + '/hdl_graph_slam/publish_graph'),
            ('/hdl_graph_slam/dump', '/' + model_namespace + '/hdl_graph_slam/dump'),
            ('/hdl_graph_slam/save_map', '/' + model_namespace + '/hdl_graph_slam/save_map'),
            ('/hdl_graph_slam/get_map', '/' + model_namespace + '/hdl_graph_slam/get_map'),
            ('/hdl_graph_slam/get_graph_estimate', '/' + model_namespace + '/hdl_graph_slam/get_graph_estimate'),
            ('/hdl_graph_slam/get_graph_gids', '/' + model_namespace + '/hdl_graph_slam/get_graph_gids'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    composable_nodes = []
    if prefiltering_params['enable_prefiltering']:
        composable_nodes.append(prefiltering_node)
    if scan_matching_odometry_params['enable_scan_matching_odometry']:
        composable_nodes.append(scan_matching_odometry_node)
    if floor_detection_params['enable_floor_detection']:
        composable_nodes.append(floor_detection_node)
    composable_nodes.append(hdl_graph_slam_node)

    # https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=composable_nodes
    )

    multi_robot_communicator_params['own_name'] = model_namespace
    multi_robot_communicator = Node(
        package='hdl_graph_slam',
        executable='multi_robot_communicator',
        name='multi_robot_communicator',
        namespace=model_namespace,
        parameters=[multi_robot_communicator_params, shared_params],
        remappings=[
            ('/hdl_graph_slam/get_graph_gids', '/' + model_namespace + '/hdl_graph_slam/get_graph_gids'),
        ],
        output='screen'
    )

    launch_description_list = [static_transform_publisher]
    if shared_params['start_rviz2']:
        launch_description_list.append(rviz2)
    # For ROS2 foxy we need to add our own clock publisher, from ROS2 humble we can publish the clock topic with ros2 bag play <bag> --clock
    if os.path.expandvars('$ROS_DISTRO') != 'humble':
        launch_description_list.append(clock_publisher_ros2)
    launch_description_list.append(map2odom_publisher_ros2)
    launch_description_list.append(container)
    launch_description_list.append(load_composable_nodes)
    if multi_robot_communicator_params['enable_multi_robot_communicator']:
        launch_description_list.append(multi_robot_communicator)

    # Return nodes
    return launch_description_list


def generate_launch_description():
    launch_description_list = []
    for param_name, _ in PARAM_MAPPING.items():
        launch_description_list.append(DeclareLaunchArgument(name=param_name, default_value=''))
    launch_description_list.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description_list)
