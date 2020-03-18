import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros1_bridge',
            node_executable='dynamic_bridge',
            arguments=['--bridge-all-topics'],
            output='screen'),
        launch_ros.actions.Node(
            package='action_bridge',
            node_executable='reverse_action_bridge_follow_joint_trajectory_node',
            output='screen')
])
