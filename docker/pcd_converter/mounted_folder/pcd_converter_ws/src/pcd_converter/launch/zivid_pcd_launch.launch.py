#!/usr/bin/env python3
"""Launch the installed pcd_converter Zivid-to-Pickit bridge.

Examples:
  ros2 launch pcd_converter zivid_pcd_launch.launch.py
  ros2 launch pcd_converter zivid_pcd_launch.launch.py \
    input_pointcloud_topic:=/zivid_camera/points/xyz \
    input_rgb_topic:=/zivid_camera/color/image_color \
    input_camera_info_topic:=/zivid_camera/color/camera_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULTS = {
    "input_pointcloud_topic": "/ZividCamera/pointcloud",
    "input_rgb_topic": "/ZividCamera/rgb",
    "input_camera_info_topic": "/ZividCamera/camera_info",
    "output_pointcloud_topic": "/simulated_camera/depth_registered/points_3d_rectified",
    "output_camera_info_topic": "/simulated_camera/depth_registered/camera_info",
    "registration_topic": "/pickit/camera_registrations",
    "output_frame_id": "simulated_camera/simulated_camera_rgb_optical_frame",
    "camera_serial": "1-simulated_camera",
    "camera_namespace": "/simulated_camera",
    "camera_type": "1",
    "camera_source": "1",
    "rgb_optical_frame": "simulated_camera/simulated_camera_rgb_optical_frame",
    "root_frame": "simulated_camera/simulated_camera_link",
    "heartbeat_timeout_seconds": "-1.0",
    "publish_hz": "2.0",
    "sync_slop_seconds": "0.1",
    "sync_queue_size": "5",
    "registration_delay_seconds": "2.0",
}


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in DEFAULTS.items()
    ]
    parameters = [{name: LaunchConfiguration(name)} for name in DEFAULTS]

    bridge = Node(
        package="pcd_converter",
        executable="zivid_to_simulated_camera",
        name="zivid_to_simulated_camera",
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )
    return LaunchDescription([*declarations, bridge])
