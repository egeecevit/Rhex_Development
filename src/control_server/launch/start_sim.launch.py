import os
import xacro
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_share_path = launch_ros.substitutions.FindPackageShare(package='tbt_otonom_sim').find('tbt_otonom_sim')
    robot_model_path = os.path.join(pkg_share_path, 'urdf/robot.urdf.xacro')
    
    # world_path = os.path.join(pkg_share_path, "worlds", 'rough_terrain.world')
    
    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="empty.world",
        description='world to launch rhex in'
    )

    world_config = LaunchConfiguration("world")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": xacro.process_file(robot_model_path).toxml(),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py",
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                    FindPackageShare('tbt_otonom_sim'),
                    'worlds',
                    world_config]),
            "verbose": "true",
        }.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "rhex",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_states_controller'],
        output='screen',
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'effort_controller'],
        output='screen'
    )

    controller_node = Node(
        package="tbt_otonom_sim",
        executable="controller_server_network.py",
        output="screen",
        parameters=[
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")}
        ]
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_host = DeclareLaunchArgument(
        name="host",
        default_value="127.0.0.1",
        description='Host address of the socket'
    )

    declare_port = DeclareLaunchArgument(
        "port",
        default_value="1256",
        description='Port of the socket'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_host,
        declare_port,
        declare_world,
        # world_config,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        load_joint_state_controller,
        load_effort_controller,
        controller_node,
    ])
