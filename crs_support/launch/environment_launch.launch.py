import os
import sys
import subprocess
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
import launch_ros.actions
from launch import some_substitutions_type
from rosidl_generator_cpp import default_value_from_type

def generate_launch_description():

    if not "tesseract_collision" in os.environ["AMENT_PREFIX_PATH"]:
        # workaround for pluginlib ClassLoader bug: manually add tesseract_collision to the AMENT_PREFIX_PATH env variable
        head, tail = os.path.split(get_package_prefix('crs_support'))
        path = os.path.join(head, 'tesseract_collision')
        os.environ["AMENT_PREFIX_PATH"] += os.pathsep + path  
    
    
#    xacro = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs.urdf.xacro')
#    xacro = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'swri_demo.urdf.xacro')
    xacro = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'swri_demo2.urdf.xacro')
    urdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs.urdf')
    urdf_preview = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'crs_preview.urdf')
    srdf = os.path.join(get_package_share_directory('crs_support'), 'urdf', 'ur10e_robot.srdf')
    gzworld = os.path.join(get_package_share_directory('crs_support'), 'worlds', 'crs.world')
    
    # create urdfs from xacro file
    cmd2 = 'xacro %s prefix:=preview/ > %s'%(xacro, urdf_preview)
    cmd3 = 'xacro %s > %s'%(xacro, urdf)        
    cmd4 = 'killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'
    cmd_dict = {cmd2 : True, cmd3: True, cmd4 : False}
    
    # check if soft link exists
    gazebo_model_path = os.path.join(os.environ['HOME'],'.gazebo', 'models', 'crs_support')
    crs_model_path = get_package_share_directory('crs_support')
    cmd1 = 'ln -s %s %s'%(crs_model_path, gazebo_model_path)
    if not os.path.exists(gazebo_model_path):
        cmd_dict[cmd1] = True        
    
    for cmd, req_ in cmd_dict.items():   
        try:       
            print('Running cmd: %s' % (cmd))    
            process = subprocess.run(cmd , shell=True, check=True, stdout=subprocess.PIPE, universal_newlines=True)       
        
        except subprocess.CalledProcessError as e:
            print(e.output)
            if req_:
                return None       
    
    
    gzserver = launch.actions.ExecuteProcess(
        cmd=['xterm', '-e', 'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '--world', gzworld],
        output='screen',   
        condition = launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('sim_robot'))
    )

    spawner1 = launch_ros.actions.Node(
        node_name='spawn_node',
        #node_namespace = [launch.substitutions.LaunchConfiguration('global_ns')],
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-x', '0', '-y', '0', '-z', '0', '-file', urdf],
        condition = launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('sim_robot'))
        )

    motion_planning_server = launch_ros.actions.Node(
        node_executable='crs_motion_planning_motion_planning_server',
        package='crs_motion_planning',
        node_name='motion_planning_server',
        #node_namespace = [launch.substitutions.LaunchConfiguration('global_ns')],
        output='screen',
        parameters=[{'urdf_path': urdf,
        'srdf_path': srdf,
        'process_planner_service': "plan_process_motion",
        'freespace_motion_service': "plan_freespace_motion",
        'trajectory_topic': "set_trajectory_test",
        'base_link_frame': "base_link",
        'world_frame': "world",
        'tool0_frame': "tool0",
        'manipulator_group': "manipulator",
        'num_steps': 20,
        'max_joint_velocity': 0.12,
        'max_joint_acceleration': 0.4,
#        'max_joint_velocity': 1.5,
#        'max_joint_acceleration': 3.0,
        'min_raster_length': 4,
        'use_gazebo_simulation_time': False,
        'set_trajopt_verbose': False}])
    
    '''  
    Example of how to push a ros namespace 
    group_action = GroupAction([
        PushRosNamespace('my_ns'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('another_launch_file'),
                                 ])
        ])
    '''

    process_planner_test_server = launch_ros.actions.Node(
        node_executable='crs_motion_planning_process_planner_test',
        package='crs_motion_planning',
        node_name='process_test_server',
        output='screen')
        
    return launch.LaunchDescription([
        # arguments
        #launch.actions.DeclareLaunchArgument('global_ns', default_value = ['crs']),
        launch.actions.DeclareLaunchArgument('sim_robot',default_value = ['True']),
        
        # environment
        launch_ros.actions.Node(
             node_name = ['env_node'],
             #node_namespace = [launch.substitutions.LaunchConfiguration('global_ns')],
             package='tesseract_monitoring',
             node_executable='tesseract_monitoring_environment_node',
             output='screen',
             parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('sim_robot'),
             'desc_param': 'robot_description',
             'robot_description': urdf,
             'robot_description_semantic': srdf}]),
        
        # gazebo
#        gzserver,
#        spawner1,

        # planning
        motion_planning_server,
        process_planner_test_server,
])
