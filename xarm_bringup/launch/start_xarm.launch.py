import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    moveit_config_package = LaunchConfiguration("moveit_config_package")

    controller_params_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"]
    )

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "moveit.rviz"]
    )

    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenoh_router',
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(moveit_config_package), "/launch", "/rsp.launch.py"]
        ),
        launch_arguments={"is_simulation": "false",}.items(),
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
                    # {'robot_description': robot_description},
                    initial_joint_controllers],
        output='screen'
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    xarm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_controller", "-c", "/controller_manager"],
    )

    delayed_xarm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[xarm_controller]
        )
    )

    xgripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xgripper_controller", "-c", "/controller_manager"],
    )
    
    delayed_xgripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[xgripper_controller]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    # move_group nodes
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("xarm_moveit_config"), "/launch", "/move_group.launch.py"]
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, {"use_sim_time": False}],
        condition=IfCondition(launch_rviz),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        zenoh_router,
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_xarm_controller_spawner,
        delayed_xgripper_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        move_group,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="xarm_moveit_config",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="xarm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="xarm_moveit_config",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="xarm_empty_world.sdf",
            description="SDF description of the world",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])