from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config =  os.path.join(
        get_package_share_directory('sort_mpc_nav'),
        'config',
        'sort_mpc_nav.rviz'
    )

    return LaunchDescription([
        Node(
            package = 'sort_mpc_nav',
            executable = 'pedestrian_sim',
            name = 'pedestrian_sim',
            output = 'screen'
        ),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'static_tf_pub',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output = 'screen'
        ),

        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            arguments = ['-d', rviz_config],
            output = 'screen'
        ),

        Node(
            package = 'sort_mpc_nav',
            executable = 'sort_tracker.py',
            name = 'sort_tracker',
            output = 'screen'
        )

        Node(
            package = 'sort_mpc_nav', 
            executable = 'mpc_planner.py',
            name = 'mpc_planner'
            output = 'screen'
        )

    ])