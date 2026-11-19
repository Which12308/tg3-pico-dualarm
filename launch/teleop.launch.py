"""Observable dry-run bridge; hardware mode is explicitly rejected by the node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="dry_run", choices=["dry_run", "hardware"],
                              description="hardware remains unavailable until the dual-arm adapter is verified"),
        DeclareLaunchArgument("publish_rate", default_value="20.0"),
        Node(package="tg3_pico_dualarm", executable="bridge_node", output="screen",
             arguments=["--mode", LaunchConfiguration("mode"),
                        "--publish-rate", LaunchConfiguration("publish_rate")]),
    ])
