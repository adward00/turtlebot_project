import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 및 디렉토리 설정
    pkg_name = 'dwb_pkg' 
    pkg_share = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # 2. 파일 경로 설정 (절대 경로 사용)
    # 맵 파일 경로
    map_file_path = '/home/mg0220/launch/src/maze.yaml'
    # 파라미터 파일 경로
    params_file = os.path.join(pkg_share, 'params', '/home/mg0220/launch/src/dwb_pkg/config/rpp_param1.yaml')
    # RViz 설정 파일 경로
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 3. Launch Configuration 설정 (무조건 시뮬레이션 모드)
    # use_sim_time을 변수로 받지 않고 그냥 'true'로 고정합니다.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')

    # 4. Arguments 선언 (필요한 것만 남김)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 5. Gazebo 실행 (무조건 실행)
    # [참고] 사용자 맵과 환경이 다를 경우 'empty_world.launch.py'로 변경 고려
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_maze3.launch.py')
        )
    )

    # 6. Nav2 Bringup 실행 (맵 서버, AMCL, 코스트맵 등)
    # 시뮬레이션에서도 길찾기를 하려면 Nav2가 반드시 필요합니다.
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',  # 강제로 true 설정
            'params_file': params_file_config,
            'autostart': 'true',  
            'use_composition': 'True' 
        }.items()
    )

    # 7. RViz 실행
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. 커스텀 A* + RPP 제어 노드 실행
    start_planner_node_cmd = Node(
        package=pkg_name,
        executable='rpp_obstacle',  
        name='a_star_rpp_planner2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. Launch Description 구성
    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(start_gazebo_cmd) 
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_planner_node_cmd)

    return ld