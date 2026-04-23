import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.conditions import IfCondition


def generate_launch_description():
    
    use_sim = LaunchConfiguration('use_sim')

    sim_arg = DeclareLaunchArgument(
        'use_sim', 
        default_value='true'
    )

    apriltag_config = os.path.join(
        get_package_share_directory('puzzlebot_vision'),
        'cfg',
        'tags_36h11.yaml'
    )

    apriltag_node = Node(   
        package='puzzlebot_vision',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[apriltag_config],
    )

    camera_noise = Node(   
        package='puzzlebot_vision',
        executable='camera_noise_node',
        condition=IfCondition(use_sim) #only activated if use sim is set
    )

    return LaunchDescription([
        sim_arg,
        apriltag_node, 
        camera_noise
    ])