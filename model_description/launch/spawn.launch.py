import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('model_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'hot_wheels.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'hot_wheels',
                   '-file', urdf_file,
                   '-x', '0.0', '-y', '0.0', '-z', '0.12'],
        output='screen'
    )

    gps_converter = Node(
        package='model_description',
        executable='gps_converter.py',
        name='gps_converter',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'reference_latitude_deg': 0.0,
                     'reference_longitude_deg': 0.0,
                     'reference_altitude_m': 0.0,
                     'position_stddev_m': 0.35}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        gps_converter,
    ])
