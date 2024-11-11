import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

# Parameter type mapping to infer the correct data type from the cli argument string. This is necessary since all cli arguments are strings.
# The parameters defined in the PARAM_MAPPING can be provided as cli arguments to overwrite the values from the yaml file.
PARAM_MAPPING = {
    'model_namespace': str,
    'use_sim_time': bool,
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
    'init_odom_topic': str,
    'init_pose_topic': str,
    'result_dir': str,
    'registration_method': str,
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


def print_remappings(remappings, header=None):
    if header is not None:
        print('######## ' + header + ' remappings ########')
    [print(remapping[0] + ' -> ' + remapping[1]) for remapping in remappings]
    print('')


def launch_setup(context, *args, **kwargs):

    config_file = 'mrg_slam.yaml'
    if 'config' in context.launch_configurations:
        config_file = context.launch_configurations['config']
    config_file_path = os.path.join(get_package_share_directory('mrg_slam'), 'config', config_file)

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)
        print('Loaded config file: ' + config_file_path)
        # # Print all parameters from the yaml file for convenience when launching the nodes
        # print(yaml.dump(config_params, sort_keys=False, default_flow_style=False))
        shared_params = config_params['/**']['ros__parameters']
        lidar2base_publisher_params = config_params['lidar2base_publisher']['ros__parameters']
        map2robotmap_publisher_params = config_params['map2robotmap_publisher']['ros__parameters']
        clock_publisher_ros2_params = config_params['clock_publisher_ros2']['ros__parameters']
        velodyne_params = config_params['velodyne']['ros__parameters']
        prefiltering_params = config_params['prefiltering_component']['ros__parameters']
        scan_matching_odometry_params = config_params['scan_matching_odometry_component']['ros__parameters']
        floor_detection_params = config_params['floor_detection_component']['ros__parameters']
        mrg_slam_params = config_params['mrg_slam_component']['ros__parameters']

    # Overwrite the parameters from the yaml file with the ones from the cli
    shared_params = overwrite_yaml_params_from_cli(shared_params, context.launch_configurations)
    lidar2base_publisher_params = overwrite_yaml_params_from_cli(lidar2base_publisher_params, context.launch_configurations)
    map2robotmap_publisher_params = overwrite_yaml_params_from_cli(map2robotmap_publisher_params, context.launch_configurations)
    velodyne_params = overwrite_yaml_params_from_cli(velodyne_params, context.launch_configurations)
    prefiltering_params = overwrite_yaml_params_from_cli(prefiltering_params, context.launch_configurations)
    scan_matching_odometry_params = overwrite_yaml_params_from_cli(scan_matching_odometry_params, context.launch_configurations)
    floor_detection_params = overwrite_yaml_params_from_cli(floor_detection_params, context.launch_configurations)
    mrg_slam_params = overwrite_yaml_params_from_cli(mrg_slam_params, context.launch_configurations)

    model_namespace = shared_params['model_namespace']

    print_yaml_params(shared_params, 'shared_params')
    print_yaml_params(lidar2base_publisher_params, 'lidar2base_publisher_params')
    print_yaml_params(map2robotmap_publisher_params, 'map2robotmap_publisher_params')
    print_yaml_params(clock_publisher_ros2_params, 'clock_publisher_ros2_params')
    print_yaml_params(velodyne_params, 'velodyne_params')
    print_yaml_params(prefiltering_params, 'prefiltering_params')
    print_yaml_params(scan_matching_odometry_params, 'scan_matching_odometry_params')
    print_yaml_params(floor_detection_params, 'floor_detection_params')
    print_yaml_params(mrg_slam_params, 'mrg_slam_params')

    # Create the static transform publisher node between the base and the lidar frame
    if lidar2base_publisher_params['enable_lidar2base_publisher']:
        frame_id = lidar2base_publisher_params['base_frame_id']
        child_frame_id = lidar2base_publisher_params['lidar_frame_id']
        if model_namespace != '':
            frame_id = model_namespace + '/' + frame_id
            child_frame_id = model_namespace + '/' + child_frame_id
        static_transform_publisher = Node(
            name='lidar2base_publisher',
            namespace=model_namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            # arguments has to be a list of strings
            arguments=[str(lidar2base_publisher_params['lidar2base_x']),
                       str(lidar2base_publisher_params['lidar2base_y']),
                       str(lidar2base_publisher_params['lidar2base_z']),
                       str(lidar2base_publisher_params['lidar2base_roll']),
                       str(lidar2base_publisher_params['lidar2base_pitch']),
                       str(lidar2base_publisher_params['lidar2base_yaw']),
                       frame_id,
                       child_frame_id],
            parameters=[shared_params],
            output='both'
        )

    # Create the map2robotmap publisher node, if it is enabled and a model_namespace is set
    if map2robotmap_publisher_params['enable_map2robotmap_publisher'] and model_namespace != '':
        map2robotmap_child_frame_id = model_namespace + '/' + map2robotmap_publisher_params['map2robotmap_child_frame_id']
        map2robotmap_publisher = Node(
            name='map2robotmap_publisher',
            namespace=model_namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            # arguments has to be a list of strings
            arguments=[str(map2robotmap_publisher_params['map2robotmap_x']),
                       str(map2robotmap_publisher_params['map2robotmap_y']),
                       str(map2robotmap_publisher_params['map2robotmap_z']),
                       str(map2robotmap_publisher_params['map2robotmap_roll']),
                       str(map2robotmap_publisher_params['map2robotmap_pitch']),
                       str(map2robotmap_publisher_params['map2robotmap_yaw']),
                       map2robotmap_publisher_params['map2robotmap_frame_id'],
                       map2robotmap_child_frame_id],
            parameters=[shared_params],
            output='both'
        )

    # In case we play a rosbag in ROS2 foxy, we need to publish the clock from the rosbag to the /clock topic
    if os.path.expandvars('$ROS_DISTRO') == 'foxy' or os.path.expandvars('$ROS_DISTRO') == 'eloquent':
        clock_publisher_ros2 = Node(
            package='mrg_slam',
            executable='clock_publisher_ros2.py',
            name=model_namespace + '_clock_publisher_ros2',
            parameters=[clock_publisher_ros2_params, shared_params],
            output='both'
        )

    # Create the map2odom publisher node
    map2odom_publisher_ros2 = Node(
        package='mrg_slam',
        executable='map2odom_publisher_ros2.py',
        name='map2odom_publisher_ros2',
        namespace=model_namespace,
        output='both',
        parameters=[mrg_slam_params, shared_params]
    )

    # Create the container node
    # If the launch command provides the debug argument we add the prefix to start gdbserver (move this to the node you need to debug)
    # More information can be found in hdl_multi_robot_graph_slam_debug.launch.py and at https://gist.github.com/JADC362/a4425c2d05cdaadaaa71b697b674425f
    if 'debug' in context.launch_configurations:
        prefix = ['gdbserver localhost:3000']
    else:
        prefix = []
    container_name = 'mrg_slam_container'
    if model_namespace != '':
        container_name = model_namespace + '/mrg_slam_container'  # used in composable nodes
    container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name="mrg_slam_container",
        namespace=model_namespace,
        output='both',
        parameters=[shared_params],
        prefix=prefix
    )

    # Launch the velodyne driver and pointcloud transform node if the lidar is enabled in the config
    # Note, we don't need the laserscan node, since we only use the pointcloud data
    # Also, we set the frame_ids according to the model_namespace, e.g. atlas/velodyne (default without namespace = velodyne)
    if velodyne_params['enable_velodyne']:
        velodyne_driver_share_dir = get_package_share_directory('velodyne_driver')
        velodyne_params_file = os.path.join(velodyne_driver_share_dir, 'config',
                                            velodyne_params['velodyne_driver']['driver_node_params_file'])
        with open(velodyne_params_file, 'r') as f:
            driver_node_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
        # overwrite the frame_id to consider the model_namespace
        if model_namespace != '':
            driver_node_params['frame_id'] = model_namespace + '/' + velodyne_params['velodyne_driver']['frame_id']
        velodyne_driver_node = ComposableNode(
            package='velodyne_driver',
            plugin='velodyne_driver::VelodyneDriver',
            name='velodyne_driver_node',
            namespace=model_namespace,
            parameters=[shared_params, driver_node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        velodyne_transform_share_dir = get_package_share_directory('velodyne_pointcloud')
        velodyne_transform_params_file = os.path.join(velodyne_transform_share_dir, 'config',
                                                      velodyne_params['velodyne_transform']['transform_node_params_file'])
        with open(velodyne_transform_params_file, 'r') as f:
            transform_node_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
        # overwrite the frame_id to consider the model_namespace, and set correct calibration file
        if model_namespace != '':
            transform_node_params['fixed_frame'] = model_namespace + '/' + velodyne_params['velodyne_transform']['fixed_frame']
        transform_node_params['calibration'] = os.path.join(velodyne_transform_share_dir, 'params',
                                                            velodyne_params['velodyne_transform']['calibration'])
        velodyne_transform_node = ComposableNode(
            package='velodyne_pointcloud',
            plugin='velodyne_pointcloud::Transform',
            name='velodyne_transform_node',
            namespace=model_namespace,
            parameters=[shared_params, transform_node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # Create the composable nodes, change names, topics, remappings to avoid conflicts for the multi robot case

    # prefiltering component
    prefiltering_remaps = [('imu/data', shared_params['imu_topic']),
                           ('velodyne_points', shared_params['points_topic'])]
    print_remappings(prefiltering_remaps, 'prefiltering_component')
    if model_namespace != '':
        prefiltering_params['base_link_frame'] = model_namespace + '/' + prefiltering_params['base_link_frame']
    if prefiltering_params['enable_prefiltering']:
        prefiltering_node = ComposableNode(
            package='mrg_slam',
            plugin='mrg_slam::PrefilteringComponent',
            name='prefiltering_component',
            namespace=model_namespace,
            parameters=[prefiltering_params, shared_params],
            remappings=prefiltering_remaps,
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # scan_matching_odometry component
    # set the correct frame ids according to the model namespace
    scan_matching_odometry_params['odom_frame_id'] = model_namespace + '/' + scan_matching_odometry_params['odom_frame_id']
    scan_matching_odometry_params['robot_odom_frame_id'] = model_namespace + '/' + scan_matching_odometry_params['robot_odom_frame_id']
    if scan_matching_odometry_params['enable_scan_matching_odometry']:
        scan_matching_odometry_node = ComposableNode(
            package='mrg_slam',
            plugin='mrg_slam::ScanMatchingOdometryComponent',
            name='scan_matching_odometry_component',
            namespace=model_namespace,
            parameters=[scan_matching_odometry_params, shared_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # helper node to write the odometry to a file
    if scan_matching_odometry_params['enable_scan_matching_odometry'] and scan_matching_odometry_params['enable_odom_to_file']:
        scan_matching_odometry_remaps = [('odom', 'scan_matching_odometry/odom')]
        odom_to_file_node = Node(
            name='odom_to_file',
            package='mrg_slam',
            executable='odom_to_file.py',
            namespace=model_namespace,
            remappings=scan_matching_odometry_remaps,
            output='screen',
            parameters=[{'result_file': '/tmp/' + model_namespace + '_scan_matching_odom.txt',
                         'every_n': 1}],
        )

    # floor_detection component
    if floor_detection_params['enable_floor_detection']:
        floor_detection_node = ComposableNode(
            package='mrg_slam',
            plugin='mrg_slam::FloorDetectionComponent',
            name='floor_detection_component',
            namespace=model_namespace,
            parameters=[floor_detection_params, shared_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # mrg_slam component
    if mrg_slam_params['enable_graph_slam']:
        mrg_slam_params['own_name'] = model_namespace
        # Overwrite init_pose array with the actual values
        mrg_slam_params['init_pose'][0] = mrg_slam_params['x']
        mrg_slam_params['init_pose'][1] = mrg_slam_params['y']
        mrg_slam_params['init_pose'][2] = mrg_slam_params['z']
        mrg_slam_params['init_pose'][3] = mrg_slam_params['roll']
        mrg_slam_params['init_pose'][4] = mrg_slam_params['pitch']
        mrg_slam_params['init_pose'][5] = mrg_slam_params['yaw']
        # set the correct frame ids according to the model namespace
        if model_namespace != '':
            mrg_slam_params['map_frame_id'] = model_namespace + '/' + mrg_slam_params['map_frame_id']
            mrg_slam_params['odom_frame_id'] = model_namespace + '/' + mrg_slam_params['odom_frame_id']
        mrg_slam_remaps = [('imu/data', shared_params['imu_topic'])]
        print_remappings(mrg_slam_remaps, 'mrg_slam_component')
        mrg_slam_node = ComposableNode(
            package='mrg_slam',
            plugin='mrg_slam::MrgSlamComponent',
            name='mrg_slam_component',
            namespace=model_namespace,
            parameters=[mrg_slam_params, shared_params],
            remappings=mrg_slam_remaps,
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    composable_nodes = []
    if velodyne_params['enable_velodyne']:
        composable_nodes.append(velodyne_driver_node)
        composable_nodes.append(velodyne_transform_node)
    if prefiltering_params['enable_prefiltering']:
        composable_nodes.append(prefiltering_node)
    if scan_matching_odometry_params['enable_scan_matching_odometry']:
        composable_nodes.append(scan_matching_odometry_node)
    if floor_detection_params['enable_floor_detection']:
        composable_nodes.append(floor_detection_node)
    if mrg_slam_params['enable_graph_slam']:
        composable_nodes.append(mrg_slam_node)

    # https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=composable_nodes
    )

    launch_description_list = []
    if lidar2base_publisher_params['enable_lidar2base_publisher']:
        launch_description_list.append(static_transform_publisher)
    if map2robotmap_publisher_params['enable_map2robotmap_publisher'] and model_namespace != '':
        launch_description_list.append(map2robotmap_publisher)
    # For ROS2 foxy we need to add our own clock publisher, from ROS2 humble we can publish the clock topic with ros2 bag play <bag> --clock
    if os.path.expandvars('$ROS_DISTRO') == 'foxy' or os.path.expandvars('$ROS_DISTRO') == 'eloquent':
        launch_description_list.append(clock_publisher_ros2)
    launch_description_list.append(map2odom_publisher_ros2)
    launch_description_list.append(container)
    launch_description_list.append(load_composable_nodes)
    if scan_matching_odometry_params['enable_scan_matching_odometry'] and scan_matching_odometry_params['enable_odom_to_file']:
        launch_description_list.append(odom_to_file_node)
    # Return nodes to our OpaqueFunction
    return launch_description_list


def generate_launch_description():
    launch_description_list = []
    # This loop enables the user to overwrite the default parameters from the yaml file with the cli arguments
    for param_name, _ in PARAM_MAPPING.items():
        launch_description_list.append(DeclareLaunchArgument(name=param_name, default_value=''))
    # With the OpaqueFunction we can access launch context in the launch_setup function
    launch_description_list.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description_list)
