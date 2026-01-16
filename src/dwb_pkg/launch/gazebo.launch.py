from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_sim = get_package_share_directory('dwb_pkg')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_maze3.world.launch.py')
        )
    )

    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_sim, 'config', 'slam_params.yaml')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        slam
    ])
