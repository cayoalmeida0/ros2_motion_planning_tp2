#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ld90_gz = get_package_share_directory("ld90_gz")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_amr_description = get_package_share_directory("amr_description")

    map_name = "tp2_map1"

    world = os.path.join(pkg_ld90_gz, "worlds", "tp2_map1.sdf")
    model_file = os.path.join(pkg_ld90_gz, "models", "ld90_gz.sdf")
    bridge_file = os.path.join(pkg_ld90_gz, "config", "bridge.yaml")
    gui_config = os.path.join(pkg_ld90_gz, "gui", "tp2_top_view.config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    results_dir = LaunchConfiguration("results_dir")

    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    resolution = LaunchConfiguration("resolution")
    inflation_radius = LaunchConfiguration("inflation_radius")

    max_iterations = LaunchConfiguration("max_iterations")
    step_size = LaunchConfiguration("step_size")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    goal_bias = LaunchConfiguration("goal_bias")
    collision_check_step = LaunchConfiguration("collision_check_step")
    smooth_iterations = LaunchConfiguration("smooth_iterations")
    seed = LaunchConfiguration("seed")

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            [
                os.path.dirname(pkg_amr_description),
                pkg_ld90_gz,
            ]
        ),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world} --gui-config {gui_config}",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ld90",
            "-file",
            model_file,
            "-x",
            start_x,
            "-y",
            start_y,
            "-z",
            "0.0",
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": bridge_file},
            {"qos_overrides./tf.publisher.durability": "volatile"},
            {"qos_overrides./tf.publisher.reliability": "reliable"},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    grid_map = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ld90_gz",
            "grid_map",
            "--sdf",
            world,
            "--map-name",
            map_name,
            "--resolution",
            resolution,
            "--inflation-radius",
            inflation_radius,
            "--start-x",
            start_x,
            "--start-y",
            start_y,
            "--goal-x",
            goal_x,
            "--goal-y",
            goal_y,
            "--output-dir",
            results_dir,
        ],
        output="screen",
    )

    rrt_planner = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ld90_gz",
            "rrt_planner",
            "--map-name",
            map_name,
            "--results-dir",
            results_dir,
            "--start-x",
            start_x,
            "--start-y",
            start_y,
            "--goal-x",
            goal_x,
            "--goal-y",
            goal_y,
            "--snap-to-free",
            "--max-iterations",
            max_iterations,
            "--step-size",
            step_size,
            "--goal-tolerance",
            goal_tolerance,
            "--goal-bias",
            goal_bias,
            "--collision-check-step",
            collision_check_step,
            "--smooth-iterations",
            smooth_iterations,
            "--seed",
            seed,
        ],
        output="screen",
    )

    path_follower = Node(
        package="ld90_gz",
        executable="path_follower",
        name="tp2_q3_rrt_path_follower",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_name": map_name},
            {"algorithm": "rrt"},
            {"results_dir": results_dir},
            {"pose_topic": "/ld90_gt_pose"},
            {"cmd_vel_topic": "/cmd_vel"},
            {"controller_k": 1.2},
            {"feedback_linearization_l": 0.25},
            {"max_linear_speed": 0.6},
            {"max_angular_speed": 1.2},
            {"waypoint_tolerance": 0.35},
            {"final_tolerance": 0.35},
            {"waypoint_stride": 1},
            {"lookahead_distance": 0.80},
            {"closest_search_window": 40},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("results_dir", default_value="results"),

            DeclareLaunchArgument("start_x", default_value="-8.5"),
            DeclareLaunchArgument("start_y", default_value="-8.5"),
            DeclareLaunchArgument("goal_x", default_value="8.5"),
            DeclareLaunchArgument("goal_y", default_value="8.5"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),

            DeclareLaunchArgument("resolution", default_value="0.10"),
            DeclareLaunchArgument("inflation_radius", default_value="0.60"),

            DeclareLaunchArgument("max_iterations", default_value="12000"),
            DeclareLaunchArgument("step_size", default_value="0.60"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.60"),
            DeclareLaunchArgument("goal_bias", default_value="0.10"),
            DeclareLaunchArgument("collision_check_step", default_value="0.05"),
            DeclareLaunchArgument("smooth_iterations", default_value="150"),
            DeclareLaunchArgument("seed", default_value="42"),

            set_gz_resource_path,
            gz_sim,
            spawn_robot,
            bridge,

            TimerAction(period=2.0, actions=[grid_map]),
            TimerAction(period=5.0, actions=[rrt_planner]),
            TimerAction(period=8.0, actions=[path_follower]),
        ]
    )
