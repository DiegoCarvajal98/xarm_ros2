from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    world_file = LaunchConfiguration("world_file")
    gazebo_pkg = LaunchConfiguration("gazebo_pkg")

    world_path = PathJoinSubstitution([FindPackageShare(gazebo_pkg), "worlds", world_file])
    
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [ros_gz_sim, "/launch", "/gz_sim.launch.py"]
            ),
            launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items()
        )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [ros_gz_sim, "/launch", "/gz_sim.launch.py"]
            ),
            launch_arguments={'gz_args': '-g -v4 '}.items()
        )

    nodes = [
        gzserver_cmd,
        gzclient_cmd,
    ]

    return nodes

def generate_launch_description():

    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="xarm_empty_world.sdf",
            description='SDF world file description.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_pkg",
            default_value="xarm_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])