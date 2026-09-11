#!/usr/bin/env python3
"""
Solution B3.3 : téléop + contrôle clavier, d'une commande. Drone déjà en vol
(décollage à la main, section 4.1).

À copier dans example_ws/src/b3_bringup/launch/, puis colcon build.
    ros2 launch b3_bringup keyboard_control.launch.py

Le clavier est lu dans le terminal où cette commande est tapée.
Pour le bonus, remplacer l'exécutable py_keyboard_control par
py_keyboard_control_safe.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # output="screen" : l'aide de la téléop (un print) s'affiche dans le terminal
    teleop = Node(
        package="b3_tools",
        executable="teleop",
        name="teleop",
        output="screen",
    )

    keyboard_control = Node(
        package="b3_python_nodes",
        executable="py_keyboard_control",
        name="keyboard_control",
        parameters=[{
            "horizontal_speed": 2.0,
            "vertical_speed": 1.0,
            "yaw_rate": 0.5,
        }]
    )

    ld.add_action(teleop)
    ld.add_action(keyboard_control)
    return ld
