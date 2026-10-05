from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from living_map_command_post.scenario import scenario_is_current


# Demo speed presets: (linear m/s, angular rad/s). "normal" is the validated
# 4 October configuration; faster presets shorten the demo.
SPEED_PRESETS = {
    "normal": (0.55, 1.0),
    "fast": (0.8, 1.4),
    "very_fast": (1.1, 1.8),
}
NORMAL_LINEAR, NORMAL_ANGULAR = SPEED_PRESETS["normal"]


def speed_preset(name: str) -> tuple[float, float]:
    key = name.strip().lower().replace("-", "_")
    if key not in SPEED_PRESETS:
        raise RuntimeError(f"Unknown speed '{name}'; choose one of {', '.join(SPEED_PRESETS)}")
    return SPEED_PRESETS[key]


# Normal-speed pure-pursuit max look-ahead per robot (matches config/nav2/*.yaml).
BASE_MAX_LOOKAHEAD = {"writer": 1.0, "executor": 0.7, "firebot": 0.7}
# Rescue robots waiting at the entrance: (namespace, spawn offset from the Writer).
RESCUE_ROBOTS = {"executor": (-2.0, -2.0), "firebot": (-2.0, 2.0)}


def nav_nodes(namespace: str, params_file: str, linear: float, angular: float) -> list[Node]:
    """Launch only the Nav2 servers LivingMap actually uses.

    Jazzy's stock navigation_launch.py does not push the namespace itself. We
    avoid that ambiguity by assigning the namespace explicitly to every node
    and using parameter files keyed by fully-qualified node names.
    """
    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [params_file],
    }
    controller_speed = {
        "FollowPath.desired_linear_vel": round(linear * (0.35 / NORMAL_LINEAR if namespace in RESCUE_ROBOTS else 1.0), 2),
        "FollowPath.rotate_to_heading_angular_vel": angular,
        # Faster driving needs a longer pure-pursuit lookahead to stay smooth.
        "FollowPath.max_lookahead_dist": round(BASE_MAX_LOOKAHEAD[namespace] + (linear - NORMAL_LINEAR), 2),
    }
    lifecycle_nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace=namespace,
            output="screen",
            parameters=[params_file, controller_speed],
        ),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace=namespace,
            output="screen",
            parameters=[params_file, {"max_rotational_vel": angular}],
        ),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", **common),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace=namespace,
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": lifecycle_nodes,
            }],
        ),
    ]


def speed_dependent_nodes(context, writer_params: str, executor_params: str, firebot_params: str) -> list[Node]:
    linear, angular = speed_preset(LaunchConfiguration("speed").perform(context))
    demo_fallback = LaunchConfiguration("demo_fallback")
    return [
        *nav_nodes("writer", writer_params, linear, angular),
        *nav_nodes("executor", executor_params, linear, angular),
        *nav_nodes("firebot", firebot_params, linear, angular),
        Node(
            package="living_map_writer",
            executable="demo_fallback",
            parameters=[{
                "use_sim_time": True,
                "max_linear_vel": round(0.5 * linear / NORMAL_LINEAR, 2),
                "max_angular_vel": round(0.8 * angular / NORMAL_ANGULAR, 2),
            }],
            condition=IfCondition(demo_fallback),
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory("living_map_sim")
    bringup_share = get_package_share_directory("living_map_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    world = os.path.join(sim_share, "worlds", "mine.sdf")
    rviz = os.path.join(sim_share, "rviz", "living_map.rviz")
    config_dir = os.path.join(bringup_share, "config")
    writer_params = os.path.join(config_dir, "nav2", "writer.yaml")
    executor_params = os.path.join(config_dir, "nav2", "executor.yaml")
    firebot_params = os.path.join(config_dir, "nav2", "firebot.yaml")
    slam_params = os.path.join(config_dir, "slam", "writer.yaml")
    events_file = os.path.join(config_dir, "events.yaml")
    gps_anchor = os.path.join(config_dir, "gps_anchor.yaml")
    rf_config = os.path.join(config_dir, "rf.yaml")
    link_config = os.path.join(config_dir, "link.yaml")
    runtime_dir = os.environ.get("LIVING_MAP_RUNTIME_DIR", "/tmp/living_map")
    beacon_store = os.path.join(runtime_dir, "beacons.json")
    scenario_dir = os.path.join(runtime_dir, "scenario")
    if scenario_is_current(scenario_dir, world, events_file):
        events_file = os.path.join(scenario_dir, "events.yaml")
        world = os.path.join(scenario_dir, "mine.sdf")
    elif os.path.exists(os.path.join(scenario_dir, "mine.sdf")):
        print(f"[living_map] Ignoring outdated scenario in {scenario_dir}: its robots or events no longer "
              "match the current world. Place the events again in the simulation harness.", file=sys.stderr)
    demo_fallback = LaunchConfiguration("demo_fallback")
    auto_explore = LaunchConfiguration("auto_explore")

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/writer/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/writer/world_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/writer/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/executor/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/executor/world_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/executor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/firebot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/firebot/world_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/firebot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "use_lifecycle_manager": "false",
            "slam_params_file": slam_params,
        }.items(),
    )

    actions = [
        DeclareLaunchArgument(
            "demo_fallback",
            default_value="false",
            description="Use deterministic Writer Nav2 waypoints instead of frontier exploration (demo recovery only)",
        ),
        DeclareLaunchArgument("auto_explore", default_value="false", description="Start exploration without dashboard controls"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz (false frees about one CPU core on small machines)"),
        DeclareLaunchArgument(
            "speed",
            default_value="normal",
            description="Robot speed preset: normal (validated), fast or very_fast",
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_share, "models")),
        # Respawned if the start-up guard has to kill a frozen Gazebo.
        ExecuteProcess(cmd=["gz", "sim", "-r", "--gui-config", os.path.join(sim_share, "gui", "mine.config"), world],
                       output="screen", respawn=True, respawn_delay=2.0),
        Node(package="living_map_bringup", executable="sim_guard", output="screen"),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridge_args,
            remappings=[
                ("/writer/scan", "/writer/scan_raw"),
                ("/executor/scan", "/executor/scan_raw"),
                ("/firebot/scan", "/firebot/scan_raw"),
                ("/clock", "/clock_raw"),
            ],
            output="screen",
        ),
        # Gazebo steps physics every 1 ms; forward the clock at 100 Hz so every
        # simulation-time node does not decode ~1000 messages per second.
        Node(package="living_map_bringup", executable="clock_throttle", parameters=[{"period_ms": 10}],
             output="screen"),
        # Completes a Nav2 bringup whose lifecycle manager lost a service reply.
        Node(package="living_map_bringup", executable="nav2_guard",
             parameters=[{"robots": ["writer", *RESCUE_ROBOTS]}], output="screen"),
        # The pose bridge converts simulated physics odometry to each spawn origin. Writer
        # starts at world (-11, 0); Executor at (-13, -2), so Executor odom origin
        # is (-2, -2) in Writer odom. Chaining through writer/odom means SLAM's
        # map->writer/odom correction also applies to the Executor.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-2", "-2", "0", "0", "0", "0", "writer/odom", "executor/odom"],
            output="screen",
        ),
        # LiDAR mounts on the 1.5x SubT X2 (Writer) and Husky A200 (Executor) base_link,
        # matching the laser links in their model.sdf files.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.405", "0", "0", "0", "writer/base_link", "writer/laser"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.3", "0", "0.48", "0", "0", "0", "executor/base_link", "executor/laser"],
            output="screen",
        ),
        # The fire robot waits at the entrance 2 m north-west of the Writer's start.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-2", "2", "0", "0", "0", "0", "writer/odom", "firebot/odom"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.3", "0", "0.48", "0", "0", "0", "firebot/base_link", "firebot/laser"],
            output="screen",
        ),
        slam_launch,
        OpaqueFunction(function=speed_dependent_nodes, args=[writer_params, executor_params, firebot_params]),
        Node(package="living_map_writer", executable="writer", parameters=[{"use_sim_time": True, "auto_explore": auto_explore}], output="screen"),
        Node(package="living_map_writer", executable="scan_frames", parameters=[{"use_sim_time": True}], output="screen"),
        Node(
            package="living_map_writer",
            executable="frontier_explorer",
            parameters=[{"use_sim_time": True}],
            condition=UnlessCondition(demo_fallback),
            output="screen",
        ),
        Node(
            package="living_map_events",
            executable="event_sensor",
            parameters=[{"use_sim_time": True, "events_file": events_file}],
            output="screen",
        ),
        Node(
            package="living_map_beacons",
            executable="beacon_manager",
            parameters=[{"use_sim_time": False, "store_path": beacon_store}],
            output="screen",
        ),
        Node(
            package="living_map_beacons",
            executable="beacon_visualizer",
            parameters=[{"use_sim_time": True, "store_path": beacon_store}],
            output="screen",
        ),
        Node(
            package="living_map_radio",
            executable="radio",
            parameters=[{
                # Radio, gateway and beacons time-stamp with the wall clock; no sim clock needed.
                "use_sim_time": False,
                "rescue_robots": list(RESCUE_ROBOTS),
                "gateway_x": -1.5,
                "gateway_y": 0.0,
                "rf_config": rf_config,
                # Rock walls of the simulated world attenuate the radio channel.
                "world_file": world,
            }],
            output="screen",
        ),
        Node(
            package="living_map_gateway",
            executable="gateway",
            parameters=[{"use_sim_time": False, "gps_anchor": gps_anchor, "fleet": list(RESCUE_ROBOTS)}],
            output="screen",
        ),
        Node(
            package="living_map_link_sim",
            executable="link_sim",
            parameters=[{"config": link_config}],
            output="screen",
        ),
        Node(
            package="living_map_command_post",
            executable="command_post",
            parameters=[{"events_file": events_file}],
            output="screen",
        ),
        # Both rescue robots drive back to the entrance after their last objective,
        # so a parked robot never blocks another's way out.
        Node(package="living_map_executor", executable="executor",
             parameters=[{"use_sim_time": True, "return_home_after_mission": True}], output="screen"),
        # Second rescue robot: same software, its own identity, topics and frames.
        Node(package="living_map_executor", executable="executor", name="firebot",
             parameters=[{"use_sim_time": True, "robot_id": "firebot", "return_home_after_mission": True}],
             output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz], parameters=[{"use_sim_time": True}], output="screen",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    return LaunchDescription(actions)
