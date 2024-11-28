#!/usr/bin/env python3
##
# @file display.launch.py
#
# @brief Provide launch file for display node.
#
# @section author_doxygen_example Author(s)
# - Created by Masaki Murooka on 2024/11/29

# Standard library
# import os

# External library
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
# from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    run_rviz = context.launch_configurations["run_rviz"]

    # rviz_config_file = os.path.join(
    #     get_package_share_directory('mujoco_tactile_sensor_plugin'),
    #     'launch',
    #     'display.rviz')

    marker_publisher_node = Node(
        package='mujoco_tactile_sensor_plugin',
        executable='MarkerPublisher',
        name='marker_publisher',
        output='screen',
        remappings=[
            ('/tactile_sensor', '/mujoco/tactile_sensor'),
            ('/marker_arr', '/mujoco/tactile_sensor/marker_arr')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_file],
        condition=IfCondition(run_rviz)
    )

    return [
        marker_publisher_node,
        rviz_node
    ]


def generate_launch_description():
    """! Generate launch description
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "run_rviz",
            default_value="True",
            description="Run rviz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
