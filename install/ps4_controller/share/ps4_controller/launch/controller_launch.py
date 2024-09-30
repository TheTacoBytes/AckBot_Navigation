from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First, launch the Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Then, launch the PS4 controller node
        Node(
            package='ps4_controller',
            executable='ps4_joy',
            name='ps4_joy',
            output='screen'
        )
    ])

