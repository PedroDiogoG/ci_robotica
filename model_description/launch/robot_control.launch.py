import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    package_name = 'model_description'

    # Utiliza launch do URDF 
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'urdf.launch.py')
        )
    )

    # Ativa Controladores 
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    # Teleop Twist Keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',
        output='screen'
    )


    # GPS sintetizado a partir da posição ruidosa do Gazebo (lat/lon/alt e XYZ).
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
        urdf_launch,
        gps_converter,
        joint_state_spawner,
        diff_drive_spawner,
        teleop_node
    ])
