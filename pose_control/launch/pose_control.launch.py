import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'model_description'
    pose_control_share = get_package_share_directory('pose_control')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tempo de simulacao'
    )

    manager_params_arg = DeclareLaunchArgument(
        'manager_params',
        default_value=os.path.join(pose_control_share, 'params', 'manager.yaml'),
        description='YAML de parametros do controle de pose (manager)'
    )

    mission_params_arg = DeclareLaunchArgument(
        'mission_params',
        default_value=os.path.join(pose_control_share, 'params', 'mission.yaml'),
        description='YAML de parametros da missao (waypoints)'
    )

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

    manager_node = Node(
        package='pose_control',
        executable='manager',
        name='manager',
        output='screen',
        parameters=[
            LaunchConfiguration('manager_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    mission_node = Node(
        package='pose_control',
        executable='mission_follower',
        name='mission_follower',
        output='screen',
        parameters=[
            LaunchConfiguration('mission_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        manager_params_arg,
        mission_params_arg,
        urdf_launch,
        joint_state_spawner,
        diff_drive_spawner,
        # teleop_node,
        # manager_node,
        # mission_node
    ])
