#!/usr/bin/env python3
"""
Nodes fournies de B3.3 : décollage automatique + téléop clavier.

    ros2 launch b3_tools takeoff_teleop.launch.py

Le clavier est lu dans le terminal où cette commande est tapée.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    takeoff = Node(
        package="b3_tools",
        executable="takeoff",
        name="takeoff",
        parameters=[{"takeoff_alt": 10.0}],
    )

    # output="screen" : l'aide de la téléop (un print) s'affiche dans le terminal
    teleop = Node(
        package="b3_tools",
        executable="teleop",
        name="teleop",
        output="screen",
    )

    ld.add_action(takeoff)
    ld.add_action(teleop)
    return ld
