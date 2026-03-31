import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.actions import Node 


def generate_launch_description():
    
    pkg_share = get_package_share_directory('puzzlebot_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    robot_description = Command(['xacro ', urdf_file])

    args = DeclareLaunchArgument(
            'use_sim_time', 
            default_value = 'false', 
            description = 'Use simulation clock if true'

        )

    robot_st_pub = Node(
            package = 'robot_state_publisher', 
            executable = 'robot_state_publisher', 
            name = 'robot_state_publisher', 
            output = 'screen', 
            parameters = [{
            'robot_description': robot_description, 
            'use_sim_time' : use_sim_time,
            }]
        )


    robot_st_pu_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )


    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        )
        


    return LaunchDescription([
            args, robot_st_pub, robot_st_pu_gui, rviz,
        ])






