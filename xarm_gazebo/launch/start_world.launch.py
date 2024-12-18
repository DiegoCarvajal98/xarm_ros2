#! /usr/bin/python3
# -*- coding utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_flower_catcher = get_package_share_directory('xarm_gazebo')

    world = LaunchConfiguration("world")

    description_pkg_name = "xarm_description"
    install_dir = get_package_share_directory(description_pkg_name)

    # gazebo_models_path = os.path.join(pkg_flower_catcher, 'models')

    # if 'GZ_SIM_RESOURCE_PATH' in os.environ:
    #     os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    # else:
    #     os.environ['GZ_SIM_RESOURCE_PATH'] = install_dir + '/share' + ':' + gazebo_models_path

    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_flower_catcher, 'worlds', 'xarm_empty_world.sdf'), ''],
        ),
        gazebo
    ])