from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launches import *

def generate_rsp_launch(moveit_config):
    """Launch file for robot state publisher (rsp)"""

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    ld.add_action(DeclareBooleanLaunchArg("sim_gazebo", default_value=False))
    ld.add_action(DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    
    moveit_config_package = "xarm_moveit_config"
    description_file = "xarm.urdf.xacro"
    
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", LaunchConfiguration("controllers_file")]
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", description_file]
            ),
            " ",
            "sim_gazebo:=",
            LaunchConfiguration("sim_gazebo"),
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
                "use_sim_time": LaunchConfiguration("sim_gazebo"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("xarm", package_name="xarm_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
