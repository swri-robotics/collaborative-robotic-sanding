# CRS Motion Planning Package

## Testing
Launch main application launch file
```
ros2 launch crs_application crs.launch.xml
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

- output_trajectory - The determined trajectory to the goal position from the start position
- success - Whether or not the planner succeeded
- message - Information about the success or failure of the attempt

#### Usage
This service can be called from the command line, an example is presented below
  ```
  ros2 service call /plan_freespace_motion crs_msgs/srv/CallFreespaceMotion "{execute: 1, target_link: 'camera_link_optical', goal_pose: {translation: {x: 0.125, y: 0.0, z: 1.6}, rotation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}}"
  ```
### Process Planner
For simple testing run the process planner test node which can call the process planner when given a service trigger
```
ros2 run crs_motion_planning crs_motion_planning_process_planner_test
```
Then in another terminal call the service trigger
```
ros2 service call /test_process_planner std_srvs/srv/Trigger
```
Rviz will first populate with a marker array of blue arrows to show the original path.

Once planning is complete  and successful 2 new marker arrays will be publised.

The first is an array of light blue spheres to denote the unreachable waypoints.

The second includes 3 different colored arrows:
- Green: Successfully planned rasters
  
- Yellow: Skipped rasters due to raster length being below specified threshold
  
- Red: Rasters that failed to pass through trajopt surface planner
