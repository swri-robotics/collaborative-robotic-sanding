# Collaborative-Robotic-Sanding
The CRS (Collaborative Robotic Sanding) Application buit in ROS2 eloquent

[![Build Status](https://github.com/swri-robotics/collaborative-robotic-sanding/workflows/CI/badge.svg)](https://github.com/swri-robotics/collaborative-robotic-sanding/actions?query=branch%3Amaster+)
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
    
- QT5 is a dependency of [ros_scxml](https://github.com/swri-robotics/ros_scxml) therefore follow the instructions provided [here](https://github.com/swri-robotics/ros_scxml)

#### Workarounds 
- None at this moment

#### Build
```
colcon build --symlink-install
```

--
### Setup
#### Data Directory
Create a soft link (shortcut) in the home directory as follows
1. cd into the `crs_process_data` package
	```
	cd crs_process_data/data/main/toolpaths
	```
  > The **main** directory holds the toolpaths for the main workcell, choose **surrogate1** if that workcell is to be used instead.

2. Create soft link
	```
	ln -s  $(pwd) $HOME/crs_data

---
### Running Application
- See the instructions [here](crs_application/README.md)

---
### Hardware dependencies
The following instructions are necessary when running on real hardware:
- Camera Driver
Go to [Framos Depth Camera](https://www.framos.com/en/industrial-depth-cameras#downloads).  Download and Extract the [tar file](https://www.framos.com/framos3d/D400e/Software/FRAMOS_D400e_Software_Package_v1-8-0_Linux64_x64.tar.gz) and then install the FRAMOS-librealsense2-2.29.8-Linux64_x64.deb debian

- UR robot driver bridging over ROS1
Coming soon ...


