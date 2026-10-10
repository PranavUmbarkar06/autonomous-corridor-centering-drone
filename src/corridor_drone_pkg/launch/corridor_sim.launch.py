import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_corridor = get_package_share_directory('corridor_drone_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_corridor, 'worlds', 'corridor_drone.sdf')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression([
        "'-s -r ' if '", headless, "' == 'true' else '-r '"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [gz_args, world]}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )

    # Controller node wrapped with a 5.0 second startup delay
    delayed_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='corridor_drone_pkg',
                executable='corridor_controller',
                name='corridor_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only without a GUI'
        ),
        gazebo,
        bridge,
        delayed_controller
    ])