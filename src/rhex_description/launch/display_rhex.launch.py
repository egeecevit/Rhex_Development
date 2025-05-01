from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ( 
    Command, 
    PathJoinSubstitution
)
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('rhex_description'),'urdf','robot.urdf.xacro')

    world = os.path.join(get_package_share_path('rhex_description'),'world', 'rhex_world.world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Add a launch argument 'lidar_enabled' to the launch file
    lidar_enabled = LaunchConfiguration('lidar_enabled')
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # Pass the launch argument 'lidar_enabled' to the xacro file
    robot_description = Command(['xacro ', urdf_path, ' lidar_enabled:=', lidar_enabled])

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="rhex_world.world",
        description='world to launch rhex in'
    )

    world_config = LaunchConfiguration("world")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #name="robot_state_publisher",
        output="screen",
        #namespace="/rhex", for namespacing
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": LaunchConfiguration('use_sim_time')}
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
        output='screen',
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,'launch','gzserver.launch.py')
        ),
        launch_arguments={'world':world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,'launch','gzclient.launch.py')
        )
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        #arguments=['-entity', 'rhex', '-topic', '/rhex/robot_description'], for namespacing
        arguments=[
            '-entity', 'rhex', 
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen',
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py",
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                    FindPackageShare('rhex_description'),
                    'world',
                    world_config]),
            "verbose": "true",
        }.items()
    )

    controller_node = Node(
        package="control_server",
        executable="controller_server_network.py",
        output="screen",
        parameters=[
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")}
        ]
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 1000}],
    )


    return LaunchDescription([
        declare_use_sim_time,
        declare_host,
        declare_port,
        declare_world,
        DeclareLaunchArgument(
            'lidar_enabled',
            default_value='false',
            description='Use lidar_enabled if true'),
        robot_state_publisher_node,
        gazebo,
        #gzserver_cmd,
        #gzclient_cmd,
        spawn_entity,
        #load_effort_controller,
        #load_joint_state_controller,
        #controller_node,
        #joint_state_publisher_node
    ])