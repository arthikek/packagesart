
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
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000,  # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen'
        ),
          Node(
            package='raspimouse',
            executable='fusionnode2',
            name='fusionnode2',
            output='screen'
        ),
    ])
