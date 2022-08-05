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

    # image_proc_remapping = Node(
    # package='stereo_image_proc',
    # namespace='stereo_image_proc',
    # executable='DisparityNode',
    # name='stereo_image_proc',
    # remappings=[
    # ('left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
    # ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
    # ('right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
    # ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
    # ]
    # )
    # Define LaunchDescription variable
    # disparity_node = [ComposableNode(
    #             package="stereo_image_proc",
    #             plugin="stereo_image_proc::DisparityNode",
    #             name='disparity_node',
    #             namespace='disparity_node',
    #             remappings=[('left/image_rect', [LaunchConfiguration('left_namespace'), '/color/left/image']),
    #             ],
    #         ),
    #          ComposableNode(
    #             package="stereo_image_proc",
    #             plugin="stereo_image_proc::PointCloudNode",
    #             name='point_cloud_node',
    #             namespace='point_cloud_node',
    #             remappings=[],
    #         ),
    #         ]

    # container = ComposableNodeContainer(
    #     condition=LaunchConfigurationNotEquals('container', ''),
    #     name='stereo_image_proc_container',
    #     namespace='stereo',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         disparity_node
    #     ]
    # )
    # lcn = LoadComposableNodes(
    #     condition=LaunchConfigurationEquals('container', ''),
    #     composable_node_descriptions=[disparity_node],
    #     target_container=LaunchConfiguration('container'),
    # )

    launch_desc = LaunchDescription(ARGUMENTS)
    launch_desc.add_action(turtlebot4_ros_ignition)
    launch_desc.add_action(turtlebot4_stereo_image_proc)
    # launch_desc.add_action(container)
    # launch_desc.add_action(lcn)

    return launch_desc