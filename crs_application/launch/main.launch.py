import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None
    
    
def generate_launch_description():
    
    # loading moveit config
    moveit_config = load_yaml('crs_application', 'config/moveit_planning_config.yaml')
    planning_scene_mntr_opts = {'planning_scene_monitor_options' : moveit_config['planning_scene_monitor_options']}
    planning_pipelines = {'planning_pipelines' : moveit_config['planning_pipelines']}
    default_planner_options = {'default_planner_options' : moveit_config['default_planner_options']}
    workspace_bounds = {'workspace_bounds': moveit_config['workspace_bounds']}
    
    robot_description_config = load_file('crs_support', 'urdf/crs.urdf')
    robot_description = {'robot_description' : robot_description_config}
    
    robot_description_semantic_config = load_file('crs_moveit2_config', 'config/crs.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}
    
    kinematics_yaml = load_yaml('crs_moveit2_config', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('crs_application', 'config/moveit_controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml }

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('crs_moveit2_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)
    
    '''
    <let name="state_machine_file" value="$(find-pkg-share crs_application)/resources/crs_process.scxml"/>
    <node pkg="crs_application" exec="crs_application_node" name="crs_main" output="screen">
        <param name="state_machine_file" value="$(var state_machine_file)"/>
    </node>    
    '''    
    state_machine_file= {'state_machine_file' : os.path.join(get_package_share_directory('crs_application'),
                                                             'resources/crs_process.scxml')}
    crs_application_node = Node(node_name = 'crs_main',
                                #prefix = 'xterm -e gdb -ex run --args',
                                package = 'crs_application',
                                node_executable= 'crs_application_node',
                                output='screen',
                                parameters=[state_machine_file,
                                            planning_scene_mntr_opts,
                                            planning_pipelines,
                                            default_planner_options,
                                            workspace_bounds,
                                            robot_description,
                                            robot_description_semantic,
                                            kinematics_yaml,
                                            ompl_planning_pipeline_config,
                                            moveit_controllers,
                                            {'planning_pipeline':'moveit2'}])
    
    return LaunchDescription([crs_application_node])

