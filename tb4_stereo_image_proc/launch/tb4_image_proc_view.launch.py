import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions.lifecycle_node import LifecycleNode 
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.actions import DeclareLaunchArgument



ARGUMENTS = [DeclareLaunchArgument('world', default_value='my_world',
                          description='A random Ignition World'),
            DeclareLaunchArgument(
            name='approximate_sync', default_value='True',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        )]


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc'
    )
    # Paths
    turtlebot4_ros_ignition_launch = PathJoinSubstitution(
    [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    turtlebot4_stereo_image_proc_launch = PathJoinSubstitution(
    [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    # ROS world config
    turtlebot4_ros_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ros_ignition_launch]),
        launch_arguments=[('world', LaunchConfiguration('world'))],
    )

    turtlebot4_stereo_image_proc= IncludeLaunchDescription(
    PythonLaunchDescriptionSource([turtlebot4_stereo_image_proc_launch]),
    launch_arguments=[('approximate_sync', LaunchConfiguration('approximate_sync'))],

    )

    launch_desc = LaunchDescription(ARGUMENTS)
    launch_desc.add_action(turtlebot4_ros_ignition)
    launch_desc.add_action(turtlebot4_stereo_image_proc)

    return launch_desc