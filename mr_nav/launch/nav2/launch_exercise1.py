import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory('mr_nav')

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'config', 'map', 'maze_jana2.yaml'),
        description='Full path to the map yaml file to load')

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav2', 'localization_launch.py')
        ),
        launch_arguments={'map': map_yaml_file}.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav2', 'navigation_launch.py')
        )
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    start_teleop_cmd = Node(
        package='mouse_teleop',
        executable='mouse_teleop',
        name='mouse_teleop',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_teleop_cmd)

    return ld