import os
import subprocess
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
import launch_ros.actions
from launch import some_substitutions_type

def generate_launch_description():

    if not "tesseract_collision" in os.environ["AMENT_PREFIX_PATH"]:
        # workaround for pluginlib ClassLoader bug: manually add tesseract_collision to the AMENT_PREFIX_PATH env variable
        head, tail = os.path.split(get_package_prefix('crs_support'))
        path = os.path.join(head, 'tesseract_collision')
        os.environ["AMENT_PREFIX_PATH"] += os.pathsep + path

    urdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs.urdf')
    srdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'ur10e_robot.srdf')
    gzworld = os.path.join(get_package_share_directory('crs_support'), 'worlds', 'crs.world')
    
    # kill any lingering gazebo instances first
    try:    
        cmd = 'killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'
        process = subprocess.run(cmd , shell=True, check=True, stdout=subprocess.PIPE, universal_newlines=True)
    except subprocess.CalledProcessError as e:
        print(e.output)
    
    gzserver = launch.actions.ExecuteProcess(
        cmd=['xterm', '-e', 'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '--world', gzworld],
        output='screen',
        condition = launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('sim_robot'))
    )

    spawner1 = launch_ros.actions.Node(
        node_name='spawn_node',
        node_namespace = [launch.substitutions.LaunchConfiguration('global_ns')],
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-x', '0', '-y', '0', '-z', '0.05', '-file', urdf],
        condition = launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('sim_robot'))
        )

    motion_planning_server = launch_ros.actions.Node(
        node_executable='crs_motion_planning_motion_planning_server',
        package='crs_motion_planning',
        node_name='motion_planning_server',
        output='screen',
        parameters=[{'urdf_path': urdf,
        'srdf_path': srdf,
        'process_planner_service': "plan_process_motion",
        'freespace_motion_service': "plan_freespace_motion",
        'trajectory_topic': "crs/set_trajectory_test",
        'base_link_frame': "base_link",
        'world_frame': "world",
        'tool0_frame': "tool0",
        'manipulator_group': "manipulator",
        'num_steps': 20,
        'max_joint_velocity': 1.5,
        'min_raster_length': 4,
        'use_gazebo_simulation_time': True,
        'set_trajopt_verbose': False}])
    
    return launch.LaunchDescription([
        # arguments
        launch.actions.DeclareLaunchArgument('global_ns'),
        launch.actions.DeclareLaunchArgument('sim_robot',default_value = ['True']),
        
        # environment
        launch_ros.actions.Node(
             node_name = ['env_node'],
             node_namespace = [launch.substitutions.LaunchConfiguration('global_ns')],
             package='tesseract_monitoring',
             node_executable='tesseract_monitoring_environment_node',
             output='screen',
             parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('sim_robot'),
             'desc_param': 'robot_description',
             'robot_description': urdf,
             'robot_description_semantic': srdf}]),
        
        # gazebo
        gzserver,
        spawner1,

        # planning
        motion_planning_server,
])
