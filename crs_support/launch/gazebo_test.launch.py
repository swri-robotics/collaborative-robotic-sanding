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
    urdf3 = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs_v3.urdf')
    srdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'ur10e_robot.srdf')
    gzworld = os.path.join(get_package_share_directory('crs_support'), 'worlds', 'crs.world')

    try:
        crs_models_dir = str(Path.home().joinpath('.gazebo', 'models', 'crs_support').resolve(strict=True))
    except FileNotFoundError: #os.path.exists(crs_models_dir):
        gazebo_path = str(Path.home().joinpath('.gazebo', 'models').resolve())
        os.mkdir(gazebo_path + "/crs_support")
        shutil.copytree(os.path.join(get_package_share_directory('crs_support'), 'meshes'), Path.home().joinpath(Path.home().joinpath('.gazebo','models','crs_support','meshes').resolve()))

    tesseract_env = launch_ros.actions.Node(
         node_name='env_node',
         package='tesseract_monitoring',
         node_executable='tesseract_monitoring_environment_node',
         output='screen',
         parameters=[{'use_sim_time': True,
         'desc_param': 'robot_description',
         'robot_description': urdf3,
         'robot_description_semantic': srdf}])

    gzserver = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '--world', gzworld],
        output='screen'
    )

    spawner1 = launch_ros.actions.Node(
        node_name='spawn_node',
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-x', '0', '-y', '0', '-z', '0.05', '-file', urdf3])

    motion_planning_server = launch_ros.actions.Node(
        node_executable='crs_motion_planning_motion_planning_server',
        package='crs_motion_planning',
        node_name='motion_planning_server',
        output='screen',
        parameters=[{'urdf_path': urdf3,
        'srdf_path': srdf,
        'process_planner_service': "plan_process_motion",
        'freespace_motion_service': "plan_freespace_motion",
        'trajectory_topic': "set_trajectory_test",
        'base_link_frame': "base_link",
        'world_frame': "world",
        'tool0_frame': "tool0",
        'manipulator_group': "manipulator",
        'num_steps': 20,
        'max_joint_velocity': 5.0,
        'min_raster_length': 4,
        'use_gazebo_simulation_time': True,
        'set_trajopt_verbose': False}])


    return launch.LaunchDescription([
        # environment
        tesseract_env,

        # gazebo
        gzserver,
        spawner1,

        # planning
        motion_planning_server,

])

