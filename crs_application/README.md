#### Run Application
**WARNING: This application uses xml launch files which is only supported in ROS Eloquent, make sure to have installed **
`apt install ros-eloquent-launch-xml`
---
- Start
    Use the *crs.launch.xml* launch file 
    ```
    ros2 launch crs_application crs.launch.xml
    ```
    
    This will start the SM and all of the required nodes.  When initialization is complete the application will go into the `Ready::Wait_User_Cmd`
    
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
