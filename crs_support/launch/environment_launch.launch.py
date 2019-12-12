import os
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
import launch_ros.actions

def generate_launch_description():

    if not "tesseract_collision" in os.environ["AMENT_PREFIX_PATH"]:
        # workaround for pluginlib ClassLoader bug: manually add tesseract_collision to the AMENT_PREFIX_PATH env variable
        head, tail = os.path.split(get_package_prefix('crs_support'))
        path = os.path.join(head, 'tesseract_collision')
        os.environ["AMENT_PREFIX_PATH"] += os.pathsep + path

    urdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs.urdf')
    srdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'ur10e_robot.srdf')

    return launch.LaunchDescription([
        # environment
        launch_ros.actions.Node(
             node_name='env_node',
             package='tesseract_monitoring',
             node_executable='tesseract_monitoring_environment_node',
             output='screen',
             parameters=[{'use_sim_time': 'true',
             'desc_param': 'robot_description',
             'robot_description': urdf,
             'robot_description_semantic': srdf}]),
])
