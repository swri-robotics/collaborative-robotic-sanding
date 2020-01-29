trajectory_msgs/JointTrajectory start				# Move prior to starting the process (empty when it does not apply)
trajectory_msgs/JointTrajectory[] process_motions	# The process moves (shall include approach and retreat moves)
trajectory_msgs/JointTrajectory[] free_motions
trajectory_msgs/JointTrajectory end					# Move after completing the process  (empty when it does not apply)