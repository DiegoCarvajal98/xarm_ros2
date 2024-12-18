from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launches import *
from launch_ros.parameter_descriptions import ParameterValue

def generate_rsp_launch(moveit_config):
    """Launch file for robot state publisher (rsp)"""

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    ld.add_action(DeclareBooleanLaunchArg("is_simulation", default_value=False))
    ld.add_action(DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    
    moveit_config_package = "xarm1s_moveit_config"
    description_file = "xarm_1s.urdf.xacro"
    
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
            "is_simulation:=",
            LaunchConfiguration("is_simulation"),
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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
                "use_sim_time": LaunchConfiguration("is_simulation"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("xarm_1s", package_name="xarm1s_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)