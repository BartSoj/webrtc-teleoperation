from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vertex_webrtc_teleop'),
        'config',
        'vertex_webrtc_teleop.yaml'
    )

    return LaunchDescription([
        Node(
            package='vertex_webrtc_teleop',
            executable='vertex_webrtc_teleop',
            name='vertex_webrtc_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
