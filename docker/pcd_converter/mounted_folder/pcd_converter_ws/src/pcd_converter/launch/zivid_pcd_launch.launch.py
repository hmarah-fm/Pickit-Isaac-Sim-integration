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
from launch_ros.parameter_descriptions import ParameterValue

from pcd_converter.configuration import DEFAULT_PARAMETERS


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=str(value))
        for name, value in DEFAULT_PARAMETERS.items()
    ]
    parameters = [{
        name: ParameterValue(LaunchConfiguration(name), value_type=type(value))
        for name, value in DEFAULT_PARAMETERS.items()
    }]

    bridge = Node(
        package="pcd_converter",
        executable="zivid_to_simulated_camera",
        name="zivid_to_simulated_camera",
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )
    return LaunchDescription([*declarations, bridge])
