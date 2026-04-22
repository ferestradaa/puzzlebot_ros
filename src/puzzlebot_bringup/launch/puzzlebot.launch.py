import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )

    use_sim = LaunchConfiguration('use_sim')
    rviz = LaunchConfiguration('rviz')

    description = get_package_share_directory('puzzlebot_description')
    control = get_package_share_directory('puzzlebot_control')
    vision = get_package_share_directory('puzzlebot_vision')


    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'rviz': rviz,
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'use_sim': use_sim,
        }.items()
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision, 'launch', 'vision.launch.py')
        )
    )

    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        desc_launch,
        control_launch,
        vision_launch
    ])