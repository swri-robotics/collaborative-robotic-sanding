# Collaborative-Robotic-Sanding
The CRS (Collaborative Robotic Sanding) Application buit in ROS2 dashing

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
    rosdep install --from-path src --ignore-src
    ```
#### Download additional resources
- colcon mixin:
    ```
    sudo apt install python3-colcon-mixin
    ```  
- QT5 is a dependency of [ros_scxml](https://github.com/swri-robotics/ros_scxml) therefore follow the instructions provided [here](https://github.com/swri-robotics/ros_scxml)

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
