import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ===== 절대 경로는 그대로 사용 =====
    params_file = '/home/mg0220/launch/src/dwb_pkg/config/rpp_param1.yaml'
    map_file = '/home/mg0220/maze.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Map yaml file'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Nav2 parameters'
    )

    # ===== Nav2 Bringup =====
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'True'
        }.items()
    )

    # ===== RViz =====
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz)

    return ld
