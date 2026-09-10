#!/usr/bin/env python3
"""
Solution B3.3 : décollage automatique + contrôle clavier.

À copier dans example_ws/src/b3_bringup/launch/, puis colcon build.
La téléop n'est pas ici : elle a besoin du clavier, et ros2 launch ne donne pas
son terminal aux nodes qu'il démarre. Elle se lance à part :
    ros2 run b3_tools teleop

Pour le bonus, remplacer l'exécutable py_keyboard_control par
py_keyboard_control_safe.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    takeoff = Node(
        package="b3_tools",
        executable="takeoff",
        name="takeoff",
        parameters=[{
            "takeoff_alt": 5.0,
            "msg_interval_rate": 10.0,
        }]
    )

    keyboard_control = Node(
        package="b3_py",
        executable="py_keyboard_control",
        name="keyboard_control",
        parameters=[{
            "horizontal_speed": 2.0,
            "vertical_speed": 1.0,
            "yaw_rate": 0.5,
        }]
    )

    ld.add_action(takeoff)
    ld.add_action(keyboard_control)
    return ld
