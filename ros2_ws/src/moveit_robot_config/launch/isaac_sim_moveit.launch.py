# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # Declare use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz in this launch file",
    )

    # Keep controller_manager and spawners configurable to avoid startup races with simulation clock.
    use_sim_time_for_control = DeclareLaunchArgument(
        "use_sim_time_for_control",
        default_value="false",
        description="Use simulation clock for ros2_control node/spawners",
    )

    controller_manager_timeout = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="60.0",
        description="Timeout (seconds) for spawners waiting for /controller_manager services",
    )

    controller_spawn_delay = DeclareLaunchArgument(
        "controller_spawn_delay",
        default_value="5.0",
        description="Delay (seconds) before spawning controllers",
    )

    spawn_aux_controllers = DeclareLaunchArgument(
        "spawn_aux_controllers",
        default_value="true",
        description="Whether to spawn joint_state_broadcaster and hand_controller",
    )

    moveit_config = (
        MoveItConfigsBuilder("mobile_base_with_franka", package_name="moveit_robot_config")
        .robot_description(
            file_path="config/mobile_base_with_franka.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/mobile_base_with_franka.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_robot_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_odom",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "odom"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_robot_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time_for_control")},
        ],
        remappings=[
            ("controller_manager/robot_description", "robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(LaunchConfiguration("spawn_aux_controllers")),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", LaunchConfiguration("controller_manager_timeout"),
            "--param-file", ros2_controllers_path,
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time_for_control")}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", LaunchConfiguration("controller_manager_timeout"),
            "--param-file", ros2_controllers_path,
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time_for_control")}],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(LaunchConfiguration("spawn_aux_controllers")),
        arguments=[
            "hand_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", LaunchConfiguration("controller_manager_timeout"),
            "--param-file", ros2_controllers_path,
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time_for_control")}],
    )

    delayed_joint_state_spawner = TimerAction(
        period=LaunchConfiguration("controller_spawn_delay"),
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_spawner = TimerAction(
        period=LaunchConfiguration("controller_spawn_delay"),
        actions=[arm_controller_spawner],
    )

    delayed_hand_spawner = TimerAction(
        period=LaunchConfiguration("controller_spawn_delay"),
        actions=[hand_controller_spawner],
    )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            use_sim_time,
            launch_rviz,
            use_sim_time_for_control,
            controller_manager_timeout,
            controller_spawn_delay,
            spawn_aux_controllers,
            rviz_node,
            world2robot_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            delayed_joint_state_spawner,
            delayed_arm_spawner,
            delayed_hand_spawner,
        ]
    )
