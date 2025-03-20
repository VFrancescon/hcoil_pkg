from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU0",
            parameters=[
                {"vConv": 0.1},
                {"iConv": 0.01},
                {"RatedV": 60},
                {"RatedI": 50},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB0"}
            ]
        ),
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU1",
            parameters=[
                {"vConv": 0.1},
                {"iConv": 0.01},
                {"RatedV": 60},
                {"RatedI": 50},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB1"}
            ]
        ),
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU2",
            parameters=[
                {"vConv": 0.01},
                {"iConv": 0.01},
                {"RatedV": 60},
                {"RatedI": 50},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB2"}
            ]
        ),
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU3",
            parameters=[
                {"vConv": 0.01},
                {"iConv": 0.01},
                {"RatedV": 60},
                {"RatedI": 50},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB3"}
            ]
        ),
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU4",
            parameters=[
                {"vConv": 0.01},
                {"iConv": 0.01},
                {"RatedV": 50},
                {"RatedI": 30},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB4"}
            ]
        ),
        Node(
            package="hcoil_pkg",
            executable="psu_node",
            name="PSU5",
            parameters=[
                {"vConv": 0.01},
                {"iConv": 0.01},
                {"RatedV": 50},
                {"RatedI": 30},
                {"debugMode": True},  # Corrected boolean parameter
                {"COM_PORT": "/dev/ttyUSB5"}
            ]
        )
    ])
