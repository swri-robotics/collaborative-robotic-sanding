#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch.frontend import Parser
from launch import LaunchService

XML_FILE_NAME = 'crs.launch.xml'
"""
@summary: entry point for ros2 launch [pkg] [launch]
@requires: ros-eloquent-launch-xml, install by running apt install ros-eloquent-launch-xml
"""
def generate_launch_description():
        
    xml_file_path = str(os.path.join(get_package_share_directory('crs_application'), 'launch', XML_FILE_NAME))
    print('Opening ROS2 launch file: %s'%(xml_file_path))
    root_entity, parser = Parser.load(xml_file_path)
    ld = parser.parse_description(root_entity)
    return ld

# main is not needed but it can be used to run as a standalone python program
if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
    
    

