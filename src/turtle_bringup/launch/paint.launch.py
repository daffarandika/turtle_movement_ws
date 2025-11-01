from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to config file
    config_file = os.path.join(
        get_package_share_directory('turtle_painter'),
        'config',
        'pid_params.yaml'
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    action_server = Node(
        package='turtle_painter',
        executable='go_to_goal',
        parameters=[
            config_file,
            {'turtle_name': 'turtle1'}
        ],
        remappings=[
            ('turtle_go_to_goal', 'turtle1/turtle_go_to_goal')
        ],
        output='screen'
    )

    delayed_action_server = TimerAction(
        period=2.0,
        actions=[action_server]
    )

    painter = Node(
        package='turtle_painter',
        executable='paint',
        output='screen'
    )

    delayed_painter = TimerAction(
        period=4.0,
        actions=[painter]
    )

    return LaunchDescription([
        turtlesim_node,
        delayed_action_server,
        delayed_painter,
    ])
