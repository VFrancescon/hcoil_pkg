from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hcoil_pkg",
            namespace="psu_debug",
            executable="psu_node",
            name="debug_psu"
        )
    ])