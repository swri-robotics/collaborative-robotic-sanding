# Hardware Setup

This application works on the Universal UR10 robot, as of now the ROS2 driver is not available therefore it is necessary to use the ROS1 bridge to allow the ROS1 robot driver to communicate with the ROS2 application

---
## Requirements
### ROS1
- Install [ROS1 melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
### Catkin
- Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

### Wstool
- Install [wstool](http://wiki.ros.org/wstool#Installation)
---
## Setting up Wokrspaces
### ROS1 Workspace
This workspace contains the robot driver node and a few other essential componets
- Create a **crs_ros1_ws** ROS 1 workspace directory
    ```
    source /opt/ros/melodic/setup.bash    # Source ROS1
    mkdir -p ~/crs_ros1_ws/src            # Make a new workspace and source directory
    cd ~/crs_ros1_ws/                     # Navigate to the workspace root
    catkin init                           # Initialize it
    ```
    > More info on catkin workspaces [here](https://catkin-tools.readthedocs.io/en/latest/quick_start.html)
- Clone the **collaborative-robotic-sanding-ros1** repository
    ```
    cd ~/crs_ros1_ws/src
    git clone https://github.com/swri-robotics/collaborative-robotic-sanding-ros1.git
    ```
- Download source dependencies with wstool
    ```
    cd ~/crs_ros1_ws/
    wstool init src src/collaborative-robotic-sanding-ros1/crs.rosinstall
    ```
- Downlad debian dependences
    ```
    rosdep install --from-path src --ignore-src -ry
    ```
    > You'll need admin priviledges to install 
- Build with catkin
    ```
    catkin build
    ```
### ROS1 Bridge Workspace
This workspace contains the bridge implementation, it allows the ROS1 and ROS2 application to communicate. 
#### WARNING!!! : **It is requried that the ROS1 amd ROS2 workspaces have been built prior to building the bridge*
- Create another workspace for the bridge
    ```
    source /opt/ros/melodic/setup.bash    # Source ROS1
    source /opt/ros/eloquent/setup.bash   # Source ROS2
    mkdir -p ~/crs_ros_bridge_ws/src            # Make a new workspace and source space
    cd ~/crs_ros_bridge_ws/                     # Navigate to the workspace root
    ```
- Clone the bridge repository
    ```
    cd ~/crs_ros_bridge_ws/                   
    git clone https://github.com/swri-robotics/crs_ros_bridge_ws.git
    ```
- Download source dependencies
    ```
    cd ~/crs_ros_bridge_ws/
    wstool init src src/crs_ros_bridge_ws/crs.rosinstall
    ```
- Downlad debian dependences
    ```
    rosdep install --from-path src --ignore-src -ry
    ```
    > You'll need admin priviledges to install 
- Overlay CRS ROS1 and ROS2 workspaces
**!!!It is assumed that your ROS2 workspace is located in the `~/crs_ws/` directory**
- Source the workspaces in the following order
    ```
    source ~/crs_ros1_ws/devel/setup.bash
    source ~/crs_ws/install/local_setup.bash
    colcon build --symlink-install
    ```
