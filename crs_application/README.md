# Run Application
**WARNING: This application uses xml launch files which is only supported in ROS Eloquent, make sure to have installed **
`apt install ros-eloquent-launch-xml`
---
## System Startup
### On Hardware
- Open terminal and navigate to ROS1 workspace
    and run the following lines in the terminal
    ```
    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
    roslaunch crs_support_ros1 crs_startup.launch
    ```
    You should see the UR teach pendant showing "Running" under the program tab
- Open another terminal window and navigate to ROS Bridge workspace
    and run the following lines in  the terminal
    ```
    source /opt/ros/melodic/setup.bash
    source /opt/ros/eloquent/setup.bash
    source install/local_setup.bash
    ros2 launch crs_bridge_support combined_bridge.launch.py
    ```
- Open a third terminal window and navigate to ROS2 workspace
    and run the following lines in the terminal
    ```
    source /opt/ros/eloquent/setup.bash
    source install/local_setup.bash
    ros2 launch crs_application crs.launch.xml sim:=false
    ```
    This last command will launch the RVIZ application where you should be able to see the robot in the mockup cell along with the user interface to control the application.

### In Simulation
- Start
    Use the *crs.launch.xml* launch file 
    ```
    ros2 launch crs_application crs.launch.xml
    ```
    
    This last command will launch the RVIZ application where you should be able to see the robot in the mockup cell along with the user interface to control the application.
    It will start the SM and all of the required nodes.  When initialization is complete the application will go into the `Ready::Wait_User_Cmd`
    
## System Usage
### Using GUI
- Select the part to be processed in the "Load Part" Screen
- On the right panel of the "Load Part" Screen select the toolpath configuration file 
- Press "Load Selected Part" button and wait for the part to load in RVIZ
- Once loaded hit "Approve" button to move to next state load part->registration->planning->execution

### Using Command line
- Query the current state
    Open a new terminal and run the following:
    ```
    ros2 topic echo /crs/current_state
    ```
    You should see the current state id printed on the terminal repeateadly 
    
- Query available actions in the current state
    From another terminal run the following
    ```
    ros2 service call /crs/get_available_actions crs_msgs/srv/GetAvailableActions 'search_pattern: "user"'
    ```
    The response may look as follows:
    ```
    response: crs_msgs.srv.GetAvailableActions_Response(action_ids=['user_done', 'user_approves'], succeeded=False, err_msg='')
    ```
    This means the the actions "user_done" and "user_approves" are valid and can be executed.
- Execute an action
    In a new terminal run the following:
    ```
    ros2 service call /crs/execute_action crs_msgs/srv/ExecuteAction 'action_id: user_approves'
    ```
    If the current state allows that action then you should see the SM transition to another state
