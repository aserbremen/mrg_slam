#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
import pprint


@launch_this(ui=False)
def mrg_slam(model_namespace: str = "", init_odom_topic: str= "", remap_tf: bool = False):
    """
    This is a launch file for the Multi-Robot Graph SLAM (mrg_slam) system.
    """
    bl = BetterLaunch()

    all_params = bl.load_params(package="mrg_slam", configfile="mrg_slam.yaml")

    shared_params = all_params["/**"]["ros__parameters"]
    model_ns = model_namespace if model_namespace else shared_params["model_namespace"]
    lidar_params = all_params["lidar2base_publisher"]["ros__parameters"]
    prefiltering_params = all_params["prefiltering_component"]["ros__parameters"]
    scan_matching_params = all_params["scan_matching_odometry_component"]["ros__parameters"]
    floor_detection_params = all_params["floor_detection_component"]["ros__parameters"]
    slam_params = all_params["mrg_slam_component"]["ros__parameters"]

    remappings = {"imu/data": shared_params["imu_topic"], "velodyne_points": shared_params["points_topic"]}
    if remap_tf:
        remappings = {**remappings, **{"/tf": "tf", "/tf_static": "tf_static"}}

    with bl.group(model_ns):
        if lidar_params["enable_lidar2base_publisher"]:
            bl.include(
                package=None,
                launchfile="static_transform_publisher.py",
                name="lidar2base_publisher",
                x=lidar_params["x"],
                y=lidar_params["y"],
                z=lidar_params["z"],
                roll=lidar_params["roll"],
                pitch=lidar_params["pitch"],
                yaw=lidar_params["yaw"],
                frame_id=model_ns + "/" + lidar_params["frame_id"] if model_ns else lidar_params["frame_id"],
                child_frame_id=model_ns + "/" + lidar_params["child_frame_id"] if model_ns else lidar_params["child_frame_id"],
                params={"use_sim_time": shared_params["use_sim_time"]},
                remaps=remappings,
            )
        
        bl.node(
            package="mrg_slam",
            executable="map2odom_publisher_ros2.py",
            name="map2odom_publisher",
            params={**shared_params, 
                    **{"map_frame_id": model_ns + "/" + slam_params["map_frame_id"] if model_ns else slam_params["map_frame_id"], 
                       "odom_frame_id": model_ns + "/" + slam_params["odom_frame_id"] if model_ns else slam_params["odom_frame_id"]}},
            remaps=remappings,
        )

        with bl.compose(name="mrg_slam_container", variant="multithreading", component_remaps=remappings, output="screen"):
            prefiltering_params["base_link_frame"] = model_ns + "/" + prefiltering_params["base_link_frame"] if model_ns else prefiltering_params["base_link_frame"]
            if prefiltering_params["enable_prefiltering"]:
                bl.component(
                    package="mrg_slam",
                    plugin="mrg_slam::PrefilteringComponent",
                    name="prefiltering_component",
                    params={**shared_params, **prefiltering_params},
                    remaps=remappings,
                )
            scan_matching_params["odom_frame_id"] = model_ns + "/" + scan_matching_params["odom_frame_id"] if model_ns else scan_matching_params["odom_frame_id"]
            scan_matching_params["robot_odom_frame_id"] = model_ns + "/" + scan_matching_params["robot_odom_frame_id"] if model_ns else scan_matching_params["robot_odom_frame_id"]
            if scan_matching_params["enable_scan_matching_odometry"]:
                bl.component(
                    package="mrg_slam",
                    plugin="mrg_slam::ScanMatchingOdometryComponent",
                    name="scan_matching_odometry_component",
                    params={**shared_params, **scan_matching_params},
                    remaps=remappings,
                )
            if floor_detection_params["enable_floor_detection"]:
                bl.component(
                    package="mrg_slam",
                    plugin="mrg_slam::FloorDetectionComponent",
                    name="floor_detection_component",
                    params={**shared_params, **floor_detection_params},
                    remaps=remappings,
                    output="screen",
                )
            slam_params["own_name"] = model_ns
            slam_params["map_frame_id"] = model_ns + "/" + slam_params["map_frame_id"] if model_ns else slam_params["map_frame_id"]
            slam_params["odom_frame_id"] = model_ns + "/" + slam_params["odom_frame_id"] if model_ns else slam_params["odom_frame_id"]
            slam_params["init_odom_topic"] = init_odom_topic if init_odom_topic else slam_params["init_odom_topic"]
            if slam_params["enable_mrg_slam"]:
                bl.component(
                    package="mrg_slam",
                    plugin="mrg_slam::MrgSlamComponent",
                    name="mrg_slam_component",
                    params={**shared_params, **slam_params},
                    remaps=remappings,
                )

