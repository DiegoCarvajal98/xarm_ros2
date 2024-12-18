import launch
import launch_ros.parameter_descriptions as launch_param_desc
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='xarm_description').find('xarm_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'xarm_1s.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/view_robot.rviz')
    # world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenoh_router',
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_param_desc.ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        zenoh_router,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])