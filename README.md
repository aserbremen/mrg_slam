# ROS2 mrg_slam package

This repository contains the source code of the `mrg_slam` package for the [Multi-Robot-Graph-SLAM](https://github.com/aserbremen/Multi-Robot-Graph-SLAM) repository. 
This package contains 4 ROS2 components, see `apps` folder:
- [prefiltering_component](apps/prefiltering_component.cpp)
- [scan_matching_odometry_component](apps/scan_matching_odometry_component.cpp)
- [floor_detection_component](apps/floor_detection_component.cpp)
- [mrg_slam_component](apps/mrg_slam_component.cpp)

For running the SLAM only using LIDAR data, the `prefiltering_component`, `scan_matching_odometry_component`, and `mrg_slam_component` are required. 
The `floor_detection_component` is optional and can be used to improve the SLAM performance, when there is a distinct floor in the environment.

Here are some things to consider when using the `mrg_slam` package:

- The launch file [mrg_slam.launch.py](launch/mrg_slam.launch.py) launches all the components required for the SLAM in a component container with intraprocess communication enabled. Plus additional nodes that are required for the SLAM to work.
- Command line arguments can be used in conjunction with the launch file to set certain parameters, such as the robot name and the initial pose of the robot.
  - The `PARAM_MAPPING` dictionary in the launch file maps the command line arguments to the parameters of the components and overwrites if they are given as command line arguments. You can remove and add parameters to the dictionary as needed.
  - All kinds of topics and services are remapped in the launch file to consider the `model_namespace` aka the robot name.
- The only required message for the SLAM to work is the `sensor_msgs/msg/PointCloud2` message with the topic `/model_namepsace/velodyne_points`. The `model_namespace` is the name of the robot, which is used to distinguish between the different robots in the system. The `frame_id` of the point cloud message should be `model_namespace/velodyne`.
- All robot names participating in the multi-robot SLAM should be given in the `multi_robot_names` parameter in the used configuration file.
  - You can use the SLAM without a `model_namespace` by setting the `model_namespace` parameter to an empty string in the configuration file. This is useful when the robot uses hard-coded frames such as `odom` or `base_link`.
  - You can also insert an empty string "" into the `multi_robot_names` parameter in the configuration file to use the SLAM without a `model_namespace` in a multi-robot scenario.
- Most nodes in the `mrg_slam.yaml` can be enabled/disabled by setting the respective parameter to `true`/`false` for testing certain parts of the SLAM system.

- Check out the [mrg_slam_velodyne_VLP16.yaml](config/mrg_slam_velodyne_VLP16.yaml) file for an example configuration file for the SLAM using live data from a Velodyne VLP-16 LIDAR sensor. `use_sim_time` is set to `false` and `velodyne/ros__parameters/enable_velodyne` is set to true in this configuration file.
  - We launch the velodyne driver node and the transform node ourselves in the launch file, because we want the `frame_id` in the point cloud message to be `model_namespace/velodyne`. The `frame_id` in the point cloud message is set to `velodyne` by default in the velodyne driver node and cannot be changed easily.
  - Also we don't need the laser scan message which is published by the velodyne driver standard launch file. We only need the point cloud message.

## Prefiltering Component

- The `prefiltering_component` is used to filter the point cloud data before it is used for the SLAM. The component subscribes to the `/model_namespace/velodyne_points` topic and publishes the filtered point cloud data on the `/model_namespace/prefiltering/filtered_points` topic.
- The `downsample_resolution` parameter for the prefiltering component can be used to downsample the point cloud data. On systems with weak computational power, it is recommended to set this parameter to a higher value to reduce the number of points in the point cloud data.

## Scan Matching Odometry Component

- The `scan_matching_odometry_component` is used to estimate the odometry of the robot using the point cloud data. The component subscribes to the `/model_namespace/prefiltering/filtered_points` topic and publishes the odometry data on the `/model_namespace/scan_matching_odometry/odom` topic.
- Odometry through scan matching is susceptible to drift over time. The `enable_imu_frontend` parameter is not tested in the ROS2 version of the package. This could potentially be used to reduce the drift in the odometry data by using the IMU data of the robot.

## Multi-Robot-Graph-SLAM Component

- The `mrg_slam_component` is used to perform the SLAM using the odometry data from the `scan_matching_odometry_component`. The component subscribes to the `/model_namespace/scan_matching_odometry/odom` and `/model_namespace/prefiltering/filtered_points` topics.
- Depending `keyframe_delta_trans` and `keyframe_delta_angle` parameters, the component decides when to add a new keyframe to the graph. 
- The graph is updated at `graph_update_interval` parameter.
- When using multiple robots, the initial poses of all robots should be set w.r.t. the same global frame.
  - For convenience, you can use the `init_odom_topic` parameter to set the initial pose of the robot using odometry messages (`nav_msgs::msg::Odometry`). Alternatively, you can set the `init_pose_topic` parameter to set the initial pose of the robot using pose messages (`geometry_msgs::msg::PoseStamped`). If you use any of these topics for multiple robots, make sure that the poses are given w.r.t. the same frame. 
  - The initial poses of the robots can also be set using the `x`, `y`, `z` (in meters) and `roll`, `pitch`, and `yaw` (in radians) parameters in the configuration file. Alternatively, the initial pose can be set using the command line arguments.
  - You can use any of the above methods to set the initial pose of the robot, where `init_odom_topic` has the highest priority, followed by `init_pose_topic`, and then the `x`, `y`, `z`, `roll`, `pitch`, and `yaw` parameters. `ros2 launch mrg_slam mrg_slam.launch.py init_odom_topic:=/robot1/odom`, or `ros2 launch mrg_slam mrg_slam.launch.py init_pose_topic:=/robot1/pose`, or `ros2 launch mrg_slam mrg_slam.launch.py x:=0 y:=0 z:=0 roll:=0 pitch:=0 yaw:=0` can be used to set the initial pose of the robot using the command line arguments.
  - Each robot performs SLAM in its own local frame. We enable a static transform broadcaster `map2robotmap_publisher` node to publish the transform between the global frame `map` and the local frame of the robot `model_namespace/map`. This way the maps of all robots can be visualized in the global frame rviz2. You can check the tf tree with `ros2 run rqt_tf_tree rqt_tf_tree`. 