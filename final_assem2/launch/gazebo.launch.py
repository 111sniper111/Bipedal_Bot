import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('final_assem2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'final_assem2.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_imu.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_desc, value_type=str),
            'use_sim_time': True,
        }]
    )

    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'bipedal_bot',
                '-z', '0.3',
                '-R', '0.0',
                '-p', '0.0',
            ],
            output='screen',
        )
    ])

    bridge = TimerAction(period=7.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen',
        )
    ])

    jsb = TimerAction(period=9.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )
    ])

    controllers = TimerAction(period=11.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['leg_position_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
    # Send standing pose automatically
    standing_pose = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/leg_position_controller/joint_trajectory',
                 'trajectory_msgs/msg/JointTrajectory',
                 '{"joint_names": ["left_white_top_joint", "left_white_bottom_joint", "right_white_top_joint", "right_white_bottom_joint"], "points": [{"positions": [0.0, 0.0, 0.0, 0.0], "velocities": [], "time_from_start": {"sec": 1, "nanosec": 0}}]}'],
            output='screen',
       )  
    ])

   

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        bridge,
        jsb,
        controllers,
        standing_pose,
    ])
