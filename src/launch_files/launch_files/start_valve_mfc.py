from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mass_flow_controller',   # First package
            executable='mass_flow_controller_node',   # Node from package_a
            name='mass_flow_controller_node'
        ),
        Node(
            package='valve_control',   # Second package
            executable='valve_controller',   # Node from package_b
            name='valve_controller'
        )
    ])
