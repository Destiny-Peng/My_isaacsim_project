from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Default to the combined URDF generated from Isaac Sim exports
    default_robot_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'combined_car_franka.urdf'
    )

    # Read URDF file content here and pass as a plain string parameter to the node
    robot_description_content = ''
    try:
        with open(default_robot_path, 'r') as f:
            robot_description_content = f.read()
    except Exception:
        robot_description_content = ''

    params = [{'robot_description': robot_description_content}, {'use_sim_time': use_sim_time}]

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=params
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('robot_file', default_value=default_robot_path, description='URDF file to load'),
        jsp_node,
        rsp_node,
    ])
