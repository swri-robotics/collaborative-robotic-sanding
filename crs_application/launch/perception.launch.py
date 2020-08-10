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
    
    # Part Localization Config
    localization_config= load_yaml_file(os.path.join(config_path, 'part_localization.yaml'))
    general_params = {'general' : localization_config['general']}
    icp_params = {'icp':localization_config['icp']}
    sac_params = {'sac':localization_config['sac']}
    crop_boxes_params = {'crop_boxes': localization_config['crop_boxes']}
    
    # Toolpath Crop Config
    region_detection_opencv_cfg = load_yaml_file(os.path.join(config_path,'toolpath_crop/region_detection_opencv_split_imgs.yaml'))
    region_detection_pcl2d_cfg = load_yaml_file(os.path.join(config_path,'toolpath_crop/region_detection_pcl2d.yaml'))
    region_detection_pcl_cfg = load_yaml_file(os.path.join(config_path,'toolpath_crop/region_detection_pcl.yaml'))
    region_crop_cfg = load_yaml_file(os.path.join(config_path,'toolpath_crop/region_crop.yaml'))
    
   
    # ComposableNodeContainer not used because it fails to load parameters, using node instead
    '''
    container =  ComposableNodeContainer(
            node_name= 'perception',
            node_namespace= GLOBAL_NS, #launch.substitutions.LaunchConfiguration('global_ns'),
            package= 'rclcpp_components',
            node_executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                package='crs_perception',
                node_plugin='crs_perception::LocalizeToPart',
                node_name='part_localization',
                node_namespace = GLOBAL_NS , #launch.substitutions.LaunchConfiguration('global_ns')),
                parameters =[icp_params,
                             sac_params,
                            crop_boxes_params 
                    ]) 
                ],
            output = 'screen'
        ) 
    '''
    
    part_localization_node = Node(
        node_executable='localize_to_part',
        package='crs_perception',
        node_name='part_localization',
        node_namespace = GLOBAL_NS ,
        output='screen',
        #prefix= 'xterm -e gdb -ex run --args',
        #prefix= 'xterm -e',
        parameters=[general_params,
                    icp_params,
                    sac_params,
                    crop_boxes_params
                    ])   
        
    region_detector_server = Node(
    node_executable='region_detector_server',
    package='region_detection_rclcpp',
    node_name='region_detector_server',
    node_namespace = GLOBAL_NS ,
    output='screen',
    #prefix= 'xterm -e gdb -ex run --args',
    prefix= 'xterm -e',
    parameters=[{'config_opencv': region_detection_opencv_cfg},
                {'config_pcl2d': region_detection_pcl2d_cfg},
                {'config_pcl': region_detection_pcl_cfg},
                ])  
    
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
    
    return [part_localization_node,
        region_detector_server,
        interactive_region_selection,
        crop_data_server]
    
def generate_launch_description():    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config_path'),
        OpaqueFunction(function = launch_setup)
        ])
