# CRS Motion Planning Package

## Testing
First run launch file that contains environment monitor, gazebo simulation, and planner nodes
  ```
  ros2 launch crs_support gazebo_test.launch.py
  ```
### Freespace Planner
#### Parameters

- target_link - TF frame of interest that will be moving to the specified goal
- start_position (optional) - specified start joint state, defaults to current state
- goal_position (optional) - specified end joint state, if unused defaults to goal_pose
- goal_pose (optional) - specified end cartesian position, required if goal_position unused
- num_steps (optional) - number of timesteps in returned trajectory, defaults to 200
- execute (optional) - if true the trajectory will be published in addition to being returned

#### Outputs

- output_trajector - The determined trajectory to the goal position from the start position
- success - Whether or not the planner succeeded
- message - Information about the success or failure of the attempt

#### Usage
This service can be called from the command line, an example is presented below
  ```
  ros2 service call /plan_freespace_motion crs_msgs/srv/CallFreespaceMotion "{execute: 1, target_link: 'camera_link_optical', goal_pose: {translation: {x: 0.125, y: 0.0, z: 1.6}, rotation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}}"
  ```
