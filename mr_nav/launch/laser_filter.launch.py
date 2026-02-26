from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_pgk = 'mr_nav'
    this_pgk_dir = get_package_share_directory(this_pgk)
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    ld = LaunchDescription()
    ld.add_action( declare_use_sim_time_cmd)
    ld.add_action( LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[{'use_sim_time': use_sim_time},
                PathJoinSubstitution([
                    this_pgk_dir, "config/laser_filter/p3dx", "shadow_filter_example.yaml",
                ])],
            remappings=[
                ("scan", "base_scan"),
                ("scan_filtered", "scan"),]
        )
    ]))
    return ld
    
