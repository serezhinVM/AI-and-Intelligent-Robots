#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # Constants for paths to different files and folders
    package_name = 'turtlebot3_gazebo'
    world_file_path = 'worlds/rosbot_sim_room_12.world'
    launch_file_path = 'launch'
    # Pose
    x_pose = '0.0'
    y_pose = '0.0'
    z_pose = '0.0'
    yaw_pose = '1.57'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    gazebo_world_path = os.path.join(pkg_share, world_file_path)
    launch_dir_path = os.path.join(pkg_share, launch_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': gazebo_world_path}.items()
    )
    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )
    # Start the Robot State Publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_path, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # Launch the robot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_path, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose
        }.items()
    )
    return LaunchDescription([
        Node(
            package='tutorial_pkg',
            executable='tutorial_pkg_node',
            name='tutorial_pkg',
            output='screen'
        ),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        start_robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
