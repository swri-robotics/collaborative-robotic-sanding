# Collaborative-Robotic-Sanding
The CRS (Collaborative Robotic Sanding) Application buit in ROS2 eloquent

[![Integration Build Status](https://travis-ci.com/swri-robotics/collaborative-robotic-sanding.svg?branch=master)](https://travis-ci.com/swri-robotics/collaborative-robotic-sanding)
[![Integration Build Status](https://travis-ci.com/swri-robotics/collaborative-robotic-sanding.svg?branch=integration)](https://travis-ci.com/swri-robotics/collaborative-robotic-sanding)
[![Github Issues](https://img.shields.io/github/issues/swri-robotics/collaborative-robotic-sanding.svg)](http://github.com/swri-robotics/collaborative-robotic-sanding/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)


---
### Workspace Setup
#### Download source dependencies
- Install [wstool](http://wiki.ros.org/wstool)
- cd into your colcon workspace root directory
- Run wstool as follows
    ```
    wstool init src src/collaborative-robotic-sanding/crs.rosinstall
    wstool update -t src
    ```
    
#### Download debian dependences
- Install [rosdep](http://wiki.ros.org/rosdep)
- From the root directory of your workspace run the following:
    ```
    rosdep install --from-path src --ignore-src -r
    ```
#### Download additional resources
- ros-eloquent-launch-xml
    ```
    sudo apt install ros-eloquent-launch-xml
    ```
    This allows using xml formatted launch files

- colcon mixin:
    ```
    sudo apt install python3-colcon-mixin
    ```  
    This is used for skipping select packages during a build, more on this later

- QT5 is a dependency of [ros_scxml](https://github.com/swri-robotics/ros_scxml) therefore follow the instructions provided [here](https://github.com/swri-robotics/ros_scxml)

#### Workarounds 
- This project uses [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [tesseract_ros2](https://github.com/ros-industrial-consortium/tesseract_ros2) which have packages with the same name. Therefore, by adding a **COLCON_IGNORE** file in the **tesseract/tesseract_ros** directory it will allow *colcon* to ignore the ROS1 packages listed in that directory.

#### Ignore select packages from colcon build
- Objective:
    Since some of the source dependencies in this workspace will have a mixture of ROS1 and ROS2 packages then these instructions will allow ignoring the ROS1/catkin packages from the build.
- Requirements:
    It is required that for a package to be ignored it must be listed in a `skip.mixin` json file inside the parent repository. The file has the following structure:
    ```
    {
      "build": {
        "skip": {
          "packages-skip": ["ros1_pkg1",
                            "ros1_pkg2",
                            "moveit_ros1",
                            ],
        }
      }
    }
    ```
- Steps:
    - Download the following script into your workspace root directory
        ```
        wget https://raw.githubusercontent.com/jrgnicho/ros_development_config/master/general/colcon_ws_setup.py
        ```
    - Run the script
        ```
        python3 colcon_ws_setup.py
        ```
    - Build the colcon workspace
        ```
        colcon build --symlink-install --mixin skip
        ```
---
### Running Application
- See the instructions [here](crs_application/README.md)

