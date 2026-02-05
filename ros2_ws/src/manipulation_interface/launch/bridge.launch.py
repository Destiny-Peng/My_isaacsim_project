from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os, sys


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic', default='arm_trajectory')
    output_topic = LaunchConfiguration('output_topic', default='arm_controller/command')

    pkg_share = get_package_share_directory('manipulation_interface')
    script_path = os.path.join(pkg_share,  'arm_trajectory_bridge.py')

    cmd = [sys.executable, script_path, '--ros-args', '--remap', f'input_topic:={input_topic}', '--remap', f'output_topic:={output_topic}']

    bridge_proc = ExecuteProcess(
        cmd=cmd,
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='arm_trajectory', description='Topic MoveIt publishes trajectories to'),
        DeclareLaunchArgument('output_topic', default_value='arm_controller/command', description='Controller command topic'),
        bridge_proc,
    ])
