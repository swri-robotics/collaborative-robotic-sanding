import os
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
import launch_ros.actions

def generate_launch_description():

    cart_sdf = os.path.join(get_package_share_directory('crs_support'), 'sdf', 'cart.sdf')

    cart_spawner = launch_ros.actions.Node(
        node_name='spawn_node',
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'cart', '-x', '0', '-y', '0.2', '-z', '0.05', '-file', cart_sdf])

    return launch.LaunchDescription([
        cart_spawner

])

