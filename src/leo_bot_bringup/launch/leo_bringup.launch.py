from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    urdf_file = os.path.join(
        os.getenv('HOME'),
        'leo_bot_ws/src/leo_bot_description/urdf/leo_bot.urdf'
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file]
        ),

        Node(
            package='leo_bot_controller',
            executable='leo_controller',
            output='screen'
        ),

    ])