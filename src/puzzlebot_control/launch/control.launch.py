import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.actions import Node 
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import SetParameter



def generate_launch_description():
    
    use_sim = LaunchConfiguration('use_sim')

    sim_arg = DeclareLaunchArgument(
            'use_sim', 
            default_value = 'false', 
        )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')


    encoders_sim = Node( #this node manually publishes the simulated angular velocity of wheels reading js
        package='puzzlebot_control', 
        executable='sim_encoders', 
        condition=IfCondition(use_sim) #only activated if use sim is set
    )
    
    js_pub = Node(
        package='puzzlebot_control', 
        executable='joint_state_pub',
        condition=UnlessCondition(use_sim) #unless because isaac sim alredady publushes js
    )   
    
    odom = Node(
        package='puzzlebot_control', 
        executable='odometry', #valid for sim or real
    )


    return LaunchDescription([
            use_sim_time_arg,
            sim_arg,
            SetParameter(name='use_sim_time', value=use_sim_time),
            js_pub,
            encoders_sim,
            odom
        ])






