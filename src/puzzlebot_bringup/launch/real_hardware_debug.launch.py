import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node

def generate_launch_description():


    description = get_package_share_directory('puzzlebot_description')
    control     = get_package_share_directory('puzzlebot_control')
    vision      = get_package_share_directory('puzzlebot_vision')

    use_sim      = LaunchConfiguration('use_sim')
    rviz         = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_arg      = DeclareLaunchArgument('use_sim',      default_value='false')
    rviz_arg         = DeclareLaunchArgument('rviz',         default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')


    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "-D", "/dev/ttyHACKER"],
        parameters=[],
        remappings=[
            ('/VelocityEncL', '/VelEncL'),
            ('/VelocityEncR', '/VelEncR'),
        ],

        output='screen'
    )

    puzzlebot_cam = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='video_source',
        output='screen',
        parameters=[{
            'resource': 'csi://0',
            'width': 640,
            'height': 360,
            'codec': 'unknown',
            'loop': 0,
            'latency': 100,
        }],
        remappings=[
            ('/video_source/raw', '/camera/image_raw'),
        ]
    )

    lidar = Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
    parameters=[{
        'channel_type': 'serial',
        'serial_port': '/dev/ttyLIDAR',
        'serial_baudrate': 115200,
        'frame_id': 'laser',
        'inverted': False,
        'angle_compensate': True,
        'scan_mode': 'Sensitivity',
    }],
    output='screen'
)

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
        micro_ros_agent,
        puzzlebot_cam,
        lidar,
        desc_launch,
        control_launch, 

    ])

