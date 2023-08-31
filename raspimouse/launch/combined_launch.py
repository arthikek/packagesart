from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      
        Node(
            package='racing_bot_controller',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='racing_bot_hat',
            executable='hat_node',
            name='hat_node',
            output='screen'
        ),
    ])
