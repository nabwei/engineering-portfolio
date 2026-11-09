from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    nav2 = Path(get_package_share_directory("nav2_bringup"))
    portfolio = Path(get_package_share_directory("portfolio_navigation"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(nav2 / "params" / "nav2_params.yaml")
            ),
            # SLAM Toolbox overrides moved to a versioned file (2026-10-28). The default
            # v1 file carries exactly the values previously inlined here; version 2 is
            # selected only by passing slam_params_file:=.../config/slam_params_v2.yaml.
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=str(portfolio / "config" / "slam_params_v1.yaml"),
            ),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-s",
                    "-r",
                    "--headless-rendering",
                    str(portfolio / "worlds" / "inspection_room.sdf"),
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0.12",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "laser",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0.14",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "base_link",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="portfolio_navigation",
                executable="velocity_guard",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    str(
                        Path(get_package_share_directory("slam_toolbox"))
                        / "config"
                        / "mapper_params_online_async.yaml"
                    ),
                    LaunchConfiguration("slam_params_file"),
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="slam_lifecycle_manager",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2 / "launch" / "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_composition": "False",
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),
        ]
    )
