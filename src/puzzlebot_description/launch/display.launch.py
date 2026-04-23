import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.actions import Node 
from launch.conditions import IfCondition
from launch_ros.actions import SetParameter


def generate_launch_description():
    
    pkg_share = get_package_share_directory('puzzlebot_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf.xacro') #loads xacro.urdf file that describes the whole robot
    robot_description = Command(['xacro ', urdf_file])
    rviz = LaunchConfiguration('rviz')

    
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    rviz_arg = DeclareLaunchArgument(
            'rviz', 
            default_value = 'false', 
        )

    rs_pub =  Node( #uses ros2 robot state publiher node for publishing joint states
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[('joint_states', '/joint_states')], #change the topic  in case using simulated joint states (not now)
        parameters=[{'robot_description': robot_description}])
                    # 'frame_prefix': 'sim/'}] use it when youve got world 
    
    rviz_node = Node( #for vialuzation, launch rviz too
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
            condition=IfCondition(rviz)
        )
        
    return LaunchDescription([
            use_sim_time_arg,
            SetParameter(name='use_sim_time', value=use_sim_time),
            rviz_arg, 
            rs_pub,
            rviz_node,
        ])






