import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node

def generate_launch_description():


    map_pub = Node(
        package='puzzlebot_navigation',
        executable='map_server_node',
        output='screen'
    )





    return LaunchDescription([

        map_pub, 

    ])

