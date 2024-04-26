#! /usr/bin/python3
# -*- coding utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_flower_catcher = get_package_share_directory('xarm_gazebo')

    description_pkg_name = "xarm_description"
    install_dir = get_package_share_directory(description_pkg_name)

    gazebo_models_path = os.path.join(pkg_flower_catcher, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share' + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_flower_catcher, 'worlds', 'xarm.world'), ''],
        ),
        gazebo
    ])