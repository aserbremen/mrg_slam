#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this(ui=False)
def static_transform_publisher(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    frame_id: str = "",
    child_frame_id: str = "",
    name: str = "static_transform_publisher",
):
    """
    Launch a static transform publisher with given arguments
    """

    bl = BetterLaunch()

    if not frame_id or not child_frame_id:
        bl.logger.error("Both frame_id and child_frame_id must be specified!")
        return

    bl.node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        cmd_args=[
            "--x",
            f"{x}",
            "--y",
            f"{y}",
            "--z",
            f"{z}",
            "--roll",
            f"{roll}",
            "--pitch",
            f"{pitch}",
            "--yaw",
            f"{yaw}",
            "--frame-id",
            f"{frame_id}",
            "--child-frame-id",
            f"{child_frame_id}",
        ],
        output="screen",
    )
