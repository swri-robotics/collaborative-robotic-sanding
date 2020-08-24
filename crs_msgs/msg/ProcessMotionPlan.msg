trajectory_msgs/JointTrajectory start				# Move prior to starting the process (empty when it does not apply)
trajectory_msgs/JointTrajectory[] process_motions	# The process moves (shall include approach and retreat moves)
cartesian_trajectory_msgs/CartesianTrajectory[] force_controlled_process_motions # Process motions as a cartesian trajectory
trajectory_msgs/JointTrajectory[] free_motions
trajectory_msgs/JointTrajectory end					# Move after completing the process  (empty when it does not apply)
