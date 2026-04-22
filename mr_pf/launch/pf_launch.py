#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    ## Extract package name
    # pkg_name = 'mr_pf'
    # this_directory = get_package_share_directory(pkg_name)

    this_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pkg_name = os.path.basename(this_directory)
            
    remappings = [('/scan', 'base_scan')]
    
    particle_filter_level_arg = DeclareLaunchArgument(
        'level', 
        default_value=TextSubstitution(text='10'), 
        description='Level for teachers')
    
    ## ToDo make it work with only the filename independet to the path from where it will be launched!
    ## Hint: you can use PathJoinSubstitution and/or OpaqueFunctions
    particle_filter_params_arg = DeclareLaunchArgument(
        'particle_filter_params_file',
        default_value=TextSubstitution(text='particle_filter.yaml'),
        description='ROS2 parameters')
    
    particle_filter_map_file_arg = DeclareLaunchArgument(
        'particle_filter_map_file',
        default_value=TextSubstitution(text='cave.png'),
        description='map image file')
    
    def get_arguments(context, *args, **kwargs):
        params_file = LaunchConfiguration('particle_filter_params_file').perform(context)
        map_file = LaunchConfiguration('particle_filter_map_file').perform(context)

        if '/' not in params_file:
            final_params_file = os.path.join(this_directory, 'config', params_file)
        else:
            final_params_file = os.path.abspath(params_file)

        if '/' not in map_file:
            final_map_file = os.path.join(this_directory, 'config', 'maps', map_file)
        else:
            final_map_file = os.path.abspath(map_file)

        return [
            SetLaunchConfiguration('particle_filter_params_file', final_params_file),
            SetLaunchConfiguration('particle_filter_map_file', final_map_file)
        ]


    return LaunchDescription([
        particle_filter_level_arg,
        particle_filter_map_file_arg,
        particle_filter_params_arg,
        OpaqueFunction(function=get_arguments),
        Node(
            package='mr_pf',
            executable='pf_node',
            name='pf',
            remappings=remappings,
            parameters=[LaunchConfiguration('particle_filter_params_file'),
                        {'alevel': LaunchConfiguration('level'),
                        'map_file': LaunchConfiguration('particle_filter_map_file')}]
        )
    ])
