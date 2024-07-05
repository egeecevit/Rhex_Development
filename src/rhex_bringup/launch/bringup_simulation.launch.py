import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    start_script_supervisor = ExecuteProcess(
        cmd=['/home/ege/development/scripts/start_rhex_supervisor.sh'],
        name='supervisor'
    )

    start_display_rhex = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rhex_description'),
            'launch',
            'display_rhex.launch.py'
        ))
    )
    
    start_launch_controller_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('control_server'),
            'launch',
            'start_controller_server.launch.py'
        ))
    )

    start_script_gui = ExecuteProcess(
        cmd=['/home/ege/development/scripts/start_fltk_gui.sh'],
        name='fltk_gui'
    )

    camera_node = Node(
        package='get_image',
        executable='obtain_image',
        output='screen'
    )

    ar_mark_node = Node(
        package="get_image",
        executable="mark_point",
        output="screen"
    )

    return LaunchDescription([
        start_script_supervisor,
        start_display_rhex,
        start_launch_controller_server,
        start_script_gui,
        #camera_node
        #ar_mark_node
    ])