from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    nav2 = Path(get_package_share_directory("nav2_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(nav2 / "params" / "nav2_params.yaml")
            ),
            Node(
                package="portfolio_navigation",
                executable="portfolio_sim",
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    str(
                        Path(get_package_share_directory("slam_toolbox"))
                        / "config"
                        / "mapper_params_online_async.yaml"
                    ),
                    {
                        "use_sim_time": False,
                        "odom_frame": "odom",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "scan_topic": "/scan",
                        "mode": "mapping",
                        "resolution": 0.05,
                        "max_laser_range": 10.0,
                        "minimum_travel_distance": 0.1,
                        "minimum_travel_heading": 0.1,
                        "use_lifecycle_manager": True,
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2 / "launch" / "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_composition": "False",
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="slam_lifecycle_manager",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                    }
                ],
            ),
        ]
    )
