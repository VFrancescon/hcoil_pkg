from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    debug_mode = LaunchConfiguration('debugMode')
    return LaunchDescription([
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU2",
            parameters=[
                {"vConv": 0.01},
                {"iConv": 0.01},
                {"RatedV": 60},
                {"RatedI": 50},
                {"debugMode": False},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB2"}
            ]
        )
    ])
