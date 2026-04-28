import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.conditions import IfCondition
from launch_ros.actions import SetParameter
from launch.conditions import UnlessCondition



def generate_launch_description():
    
    use_sim = LaunchConfiguration('use_sim')

    sim_arg = DeclareLaunchArgument(
        'use_sim', 
        default_value='false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    apriltag_config = os.path.join(
        get_package_share_directory('puzzlebot_vision'),
        'cfg',
        'tags_36h11.yaml'
    )

    apriltag_node = Node(   
        package='puzzlebot_vision',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image_rect'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[apriltag_config],
    )

    camera_noise = Node(   
        package='puzzlebot_vision',
        executable='camera_noise_node',
        condition=IfCondition(use_sim) #only activated if use sim is set
    )

    camera_info = Node(
        package = 'puzzlebot_vision',
        executable = 'camera_info_publisher',
        condition =UnlessCondition(use_sim) 

    )

    fid_stamper = Node(
        package = 'puzzlebot_vision',
        executable = 'frame_id_stamper', 
        condition =UnlessCondition(use_sim)
        
        
    )

    return LaunchDescription([
        use_sim_time_arg,
        sim_arg,
        SetParameter(name='use_sim_time', value=use_sim_time),
        apriltag_node, 
        camera_noise, 
        camera_info,
        fid_stamper,  

    ])