import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

def generate_launch_description():

    description = get_package_share_directory('puzzlebot_description')
    control     = get_package_share_directory('puzzlebot_control')

    use_sim      = LaunchConfiguration('use_sim')
    rviz         = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_arg      = DeclareLaunchArgument('use_sim',      default_value='true')  
    rviz_arg         = DeclareLaunchArgument('rviz',         default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')  


    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description, 'launch', 'display.launch.py')),
        launch_arguments={'rviz': rviz, 'use_sim_time': use_sim_time}.items()
    )
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control, 'launch', 'control.launch.py')),
        launch_arguments={'use_sim': use_sim, 'use_sim_time': use_sim_time}.items()
    )


    

    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=use_sim_time), 
        desc_launch,
        control_launch,
    ])