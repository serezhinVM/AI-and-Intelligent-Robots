#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    pkg_gazebo_ros        = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tutorial          = get_package_share_directory('tutorial_pkg')

    urdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', model_folder, 'model.sdf')
    world     = os.path.join(pkg_tutorial, 'worlds', 'rosbot_sim_room_12.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.3')
    y_pose = LaunchConfiguration('y_pose', default='0.3')
    yaw    = LaunchConfiguration('yaw',    default='1.5708')  # pi/2 → facing +Y

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
        output='screen',
    )

    # Start the navigation node a few seconds after spawn so odom is ready.
    tutorial_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='tutorial_pkg_node',
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.3'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.3'))
    ld.add_action(DeclareLaunchArgument('yaw',    default_value='1.5708'))
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(tutorial_node)
    return ld
