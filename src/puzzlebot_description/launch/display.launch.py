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

    js_pub_sim =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[('joint_states', '/joint_states')],
        parameters=[{'robot_description': robot_description}])
                    # 'frame_prefix': 'sim/'}] use it when youve got world 
    
    '''
    js_pub_real =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='real',
        remappings=[('joint_states', '/joint_states_sim')],
        parameters=[{'robot_description': robot_description}]
    )
    '''


    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        )
        


    return LaunchDescription([
            args, js_pub_sim, rviz,
        ])






