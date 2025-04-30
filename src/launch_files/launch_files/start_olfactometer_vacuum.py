from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mass_flow_controller',
            executable='mass_flow_controller_node',
            name='mass_flow_controller_node'
        ),
        Node(
            package='valve_control',
            executable='valve_controller',
            name='valve_controller'
        ),
        Node(
            package='olfacto_web_control',
            executable='vacuum_server',
            name='vacuum_server'
        )
    ])
