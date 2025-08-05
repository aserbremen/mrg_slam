#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
import pprint


@launch_this(ui=True)
def mrg_slam(model_namespace_: str = ""):
    """
    This is a launch file for the Multi-Robot Graph SLAM (mrg_slam) system.
    """
    bl = BetterLaunch()

    all_params = bl.load_params(package="mrg_slam", configfile="mrg_slam.yaml")

    shared_params = all_params["/**"]["ros__parameters"]
    model_namespace = model_namespace_ if model_namespace_ else shared_params["model_namespace"]
    lidar_params = all_params["lidar2base_publisher"]["ros__parameters"]

    remappings = []  # dict{"/tf": "/tf", "/tf_static": "/tf_static"}

    pprint.pprint(all_params)

    with bl.group(model_namespace):
        if lidar_params["enable_lidar2base_publisher"]:
            bl.include(
                package=None,
                launchfile="static_transform_publisher.py",
                x=lidar_params["x"],
                y=lidar_params["y"],
                z=lidar_params["z"],
                roll=lidar_params["roll"],
                pitch=lidar_params["pitch"],
                yaw=lidar_params["yaw"],
                frame_id=lidar_params["frame_id"],
                child_frame_id=lidar_params["child_frame_id"],
            )

        with bl.compose(name="mrg_slam_container", variant="multithreading", component_remaps=remappings):
            bl.component(
                package="mrg_slam",
                plugin="mrg_slam::PrefilteringComponent",
                name="prefiltering_component",
                params={**shared_params, **all_params["prefiltering_component"]["ros__parameters"]},
            )

    # with bl.compose("mrg_slam_container"):
    #     bl.component("composition", "")
