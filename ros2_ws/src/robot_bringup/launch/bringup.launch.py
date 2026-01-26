from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_file = LaunchConfiguration('robot_file')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'robot_description_launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'robot_file': robot_file}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('robot_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_description'), 'urdf', 'combined_car_franka.urdf'
        ]), description='URDF file to load'),
        robot_description_launch,
    ])
