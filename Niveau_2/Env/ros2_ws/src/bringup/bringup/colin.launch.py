# mission/launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Optional namespace for all nodes
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Namespace to push for all nodes (leave empty for none).'
    )

    ns = LaunchConfiguration('ns')

    # Nodes (assumes console_scripts are named like the files without .py)
    approach = Node(
        package='mission',
        executable='approach',          # -> mission/approach.py : main()
        name='approach',
        output='screen',
        # --- single remap example (only here) ---
        # Remap whatever your node subscribes/publishes.
        # Example: remap /target_pose -> /vision/target_pose
        remappings=[('/approach_target_graph', '/Balloon_pos')],
    )

    colin_drone_node = Node(
        package='mission',
        executable='colin_drone_node',  # -> mission/colin_drone_node.py : main()
        name='colin_drone_node',
        output='screen',
    )

    ballon_pub = Node(
        package='mission_control',
        executable='ballon_pub',        # -> mission_control/ballon_pub.py : main()
        name='ballon',
        output='screen',
    )

    monitor = Node(
        package='mission_control',
        executable='monitor',           # -> mission_control/monitor.py : main()
        name='monitor',
        output='screen',
    )

    group = GroupAction([
        PushRosNamespace(ns),
        approach,
        colin_drone_node,
        ballon_pub,
        monitor,
    ])

    return LaunchDescription([ns_arg, group])
