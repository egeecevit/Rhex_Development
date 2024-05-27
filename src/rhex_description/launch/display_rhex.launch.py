from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('rhex_description'),'urdf','robot.urdf.xacro')
    control_path = os.path.join(get_package_share_path('rhex_description'), 'config', 'rhex_ros_control.yaml')
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



    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #name="robot_state_publisher",
        #output="screen",
        #namespace="/rhex", for namespacing
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": LaunchConfiguration('use_sim_time')}
        ]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        #namespace="/rhex" for namespacing
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
            '-z' '1.75'
        ],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    controller_Node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='rhex'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 1000}],
    )



    return LaunchDescription([
        declare_use_sim_time,
        DeclareLaunchArgument(
            'lidar_enabled',
            default_value='false',
            description='Use lidar_enabled if true'),
        robot_state_publisher_node,
        #gazebo,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
        load_effort_controller,
        load_joint_state_controller,
        #joint_state_publisher_gui_node
        joint_state_publisher_node
        #yarrak ye
    ])