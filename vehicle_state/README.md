Vehicle lane Position
	vehicle_lane_position is a ROS package written in Python that   calculate the position of the vehicle with respect to the center of the lane.

Inputs
path information.
GPS raw data.
Outputs
Yaw error of the vehicle
Distance from the right lane
Distance from the left lane 

How to use the package
	1- clone “road_processing_planning” package and run this command in a new terminal “roslaunch road_processing_planning route_points.launch”. 

	2- from the terminal run the following command “roslaunch vehicle_state vehicle_state.launch”. 
 	
	3- subscribe to the topic “/fix” the GPS raw data by downloading the bag file which has the recorded data and running this command in its directory “ rosbag play -s 300 leganes1_2018-11-15-12-30-36.bag”.
	
	4- subscribe to the topic “/vehicle_lane_pos” to get yaw error, Rd and Ld.

What does the package do
	1- subscribe the path points to the “/path_getter” service
	2- subscribe the GPS raw data to the topic “/fix”
	3- calculate yaw error, Rd and Ld
	4- publish the the results to “/vehicle_lane_pos”  topic

