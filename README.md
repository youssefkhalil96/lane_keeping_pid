Vehicle lane position:

vehicle_state is a ROS package written in Python that calculate the position of the vehicle with respect to center of the lane.

How to use the package:

1-extract road information that you want to know the position of the vehicle from mustafa's package

2-assign the starting position of the vehicle by specifying x_init and y_init to the file "vehicle_state/scripts/vehicle_ lane_position.py"

3-from the terminal run the following command $ roslaunch vehicle_state vehicle_state.launch

4-subscribe to the topic "/vehicle_lane_pos"

5-note that the lane width is assumed to be three meters

Package Inputs:
1-x_init
2-y_init

Package Outputs:
1-yaw error
2-distance from rigth lane 
3-distance from left lane
