import launch
import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory, get_package_prefix

GLOBAL_NS = '/crs'  # WORKAROUND to failure in ComposableNodeContainer to used namespace pushed by calling launch file
    
def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None      
    
def launch_setup(context, *args, **kwargs):
    
    ## getting path
    config_path = launch.substitutions.LaunchConfiguration('config_path').perform(context)
    
    # Load configuration files
    region_detection_cfg_file = os.path.join(config_path,'rework/region_detection.yaml')
    region_crop_cfg = load_yaml_file(os.path.join(config_path,'rework/region_crop.yaml'))
  
    # Nodes 
    region_detector_server = Node(
    node_executable='region_detector_server',
    package='region_detection_rclcpp',
    node_name='region_detector_server',
    node_namespace = GLOBAL_NS ,
    output='screen',
    #prefix= 'xterm -e gdb -ex run --args',
    prefix= 'xterm -e',
    parameters=[{'region_detection_cfg_file': region_detection_cfg_file}])  
    
    interactive_region_selection = Node(
    node_executable='interactive_region_selection',
    package='region_detection_rclcpp',
    node_name='interactive_region_selection',
    node_namespace = GLOBAL_NS ,
    output='screen',
    #prefix= 'xterm -e gdb -ex run --args',
    prefix= 'xterm -e',
    parameters=[{'region_height': 0.05}
                ])  
    
    crop_data_server = Node(
        node_executable='crop_data_server',
        package='region_detection_rclcpp',
        node_name='crop_data_server',
        node_namespace = GLOBAL_NS ,
        output='screen',
        #prefix= 'xterm -e gdb -ex run --args',
        prefix= 'xterm -e',
        parameters=[{'region_crop': region_crop_cfg}
                    ],
        remappings = [('crop_data','crop_toolpaths')]
        ) 
    
    return [region_detector_server,
        interactive_region_selection,
        crop_data_server]
    
def generate_launch_description():    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config_path'),
        OpaqueFunction(function = launch_setup)
        ])
