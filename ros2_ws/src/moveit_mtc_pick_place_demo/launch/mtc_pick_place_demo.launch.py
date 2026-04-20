from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_for_control = LaunchConfiguration("use_sim_time_for_control")
    controller_spawn_delay = LaunchConfiguration("controller_spawn_delay")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")
    mtc_start_delay = LaunchConfiguration("mtc_start_delay")
    enable_gripper_actions = LaunchConfiguration("enable_gripper_actions")
    enable_initial_open_hand_stage = LaunchConfiguration("enable_initial_open_hand_stage")
    enable_move_to_pick_stage = LaunchConfiguration("enable_move_to_pick_stage")
    enable_move_to_place_stage = LaunchConfiguration("enable_move_to_place_stage")
    execute = LaunchConfiguration("execute")
    spawn_aux_controllers = LaunchConfiguration("spawn_aux_controllers")
    launch_mtc_rviz = LaunchConfiguration("launch_mtc_rviz")
    mtc_rviz_config = LaunchConfiguration("mtc_rviz_config")
    pick_x = LaunchConfiguration("pick_x")
    pick_y = LaunchConfiguration("pick_y")
    pick_z = LaunchConfiguration("pick_z")
    place_x = LaunchConfiguration("place_x")
    place_y = LaunchConfiguration("place_y")
    place_z = LaunchConfiguration("place_z")
    object_size_x = LaunchConfiguration("object_size_x")
    object_size_y = LaunchConfiguration("object_size_y")
    object_size_z = LaunchConfiguration("object_size_z")

    moveit_config = (
        MoveItConfigsBuilder("mobile_base_with_franka", package_name="moveit_robot_config")
        .robot_description(
            file_path="config/mobile_base_with_franka.urdf.xacro",
            mappings={"ros2_control_hardware_type": "isaac"},
        )
        .robot_description_semantic(file_path="config/mobile_base_with_franka.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moveit_robot_config"),
                "launch",
                "isaac_sim_moveit.launch.py",
            )
        ),
        launch_arguments={
            "ros2_control_hardware_type": "isaac",
            "use_sim_time": use_sim_time,
            "launch_rviz": "false",
            "use_sim_time_for_control": use_sim_time_for_control,
            "spawn_aux_controllers": spawn_aux_controllers,
            "controller_spawn_delay": controller_spawn_delay,
            "controller_manager_timeout": controller_manager_timeout,
        }.items(),
    )

    mtc_node = Node(
        package="moveit_mtc_pick_place_demo",
        executable="mtc_pick_place_demo",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("moveit_mtc_pick_place_demo"),
                "config",
                "pick_place_params.yaml",
            ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
                "pick_x": pick_x,
                "pick_y": pick_y,
                "pick_z": pick_z,
                "place_x": place_x,
                "place_y": place_y,
                "place_z": place_z,
                "object_size_x": object_size_x,
                "object_size_y": object_size_y,
                "object_size_z": object_size_z,
                "enable_gripper_actions": enable_gripper_actions,
                "enable_initial_open_hand_stage": enable_initial_open_hand_stage,
                "enable_move_to_pick_stage": enable_move_to_pick_stage,
                "enable_move_to_place_stage": enable_move_to_place_stage,
                "execute": execute,
            },
        ],
    )

    delayed_mtc_node = TimerAction(
        period=mtc_start_delay,
        actions=[mtc_node],
    )

    mtc_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_mtc_rviz),
        additional_env={
            "AMENT_PREFIX_PATH": f"{get_package_prefix('moveit_task_constructor_visualization')}:{os.environ.get('AMENT_PREFIX_PATH', '')}",
            "LD_LIBRARY_PATH": f"{os.path.join(get_package_prefix('moveit_task_constructor_visualization'), 'lib')}:{os.environ.get('LD_LIBRARY_PATH', '')}",
        },
        output="log",
        arguments=["-d", mtc_rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_sim_time_for_control", default_value="false"),
            DeclareLaunchArgument("controller_spawn_delay", default_value="5.0"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("mtc_start_delay", default_value="8.0"),
            DeclareLaunchArgument("enable_gripper_actions", default_value="false"),
            DeclareLaunchArgument("enable_initial_open_hand_stage", default_value="false"),
            DeclareLaunchArgument("enable_move_to_pick_stage", default_value="true"),
            DeclareLaunchArgument("enable_move_to_place_stage", default_value="true"),
            DeclareLaunchArgument("execute", default_value="true"),
            DeclareLaunchArgument("spawn_aux_controllers", default_value="true"),
            DeclareLaunchArgument("launch_mtc_rviz", default_value="true"),
            DeclareLaunchArgument(
                "mtc_rviz_config",
                default_value=os.path.join(
                    get_package_share_directory("moveit_mtc_pick_place_demo"),
                    "config",
                    "mtc_pick_place.rviz",
                ),
            ),
            DeclareLaunchArgument("pick_x", default_value="0.55"),
            DeclareLaunchArgument("pick_y", default_value="0.0"),
            DeclareLaunchArgument("pick_z", default_value="0.20"),
            DeclareLaunchArgument("place_x", default_value="0.45"),
            DeclareLaunchArgument("place_y", default_value="-0.25"),
            DeclareLaunchArgument("place_z", default_value="0.20"),
            DeclareLaunchArgument("object_size_x", default_value="0.04"),
            DeclareLaunchArgument("object_size_y", default_value="0.04"),
            DeclareLaunchArgument("object_size_z", default_value="0.12"),
            moveit_launch,
            mtc_rviz_node,
            delayed_mtc_node,
        ]
    )
