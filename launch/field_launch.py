from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Include the psu_array_launch.py file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hcoil_pkg"),
                "launch",
                "psu_array_launch.py"
            ])
            ),
            launch_arguments={
            "debugMode": "True"
            }.items()
        ),

        Node(
            package="hcoil_pkg",
            executable="field_node",
            name="field",
            parameters=[
                {"xNum": 2},
                {"yNum": 1},
                {"zNum": 1},
                {"xRoot": "PSU0"},
                {"yRoot": "PSU4"},
                {"zRoot": "PSU2"},
            ]
        )
    ])