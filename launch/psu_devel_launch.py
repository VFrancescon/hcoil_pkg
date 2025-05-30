from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hcoil_pkg",
            namespace="psu_debug",
            executable="psu_node",
            name="test_diff",
            parameters=[
                {"vConv": 0.25},
                {"iConv": 0.1},
                {"debugMode": False}
            ]
        )
    ])