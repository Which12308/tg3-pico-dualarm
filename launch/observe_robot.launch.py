"""Dry-run telemetry plus subscriptions to actual feedback; no actuator outputs."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="tg3_pico_dualarm", executable="bridge_node", output="screen",
             arguments=["--mode", "dry_run", "--observe-robot"]),
    ])
