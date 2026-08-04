from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic',
                'robot_description',
                '-name',
                'bipedal_bot',
                '-z',
                '0.3'
            ],
            output='screen'
        )

    ])
