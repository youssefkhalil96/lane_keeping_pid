Lane keeping Control

lane_keeping_pid is a Ros package written in Python to calculate the desired steering angle to keep the vehicle in middle of the lane.

Inputs

1- path information
2- gps raw data

outputs

steering angle

How to use the package:

1-This package simulate the results on CARLA so we need first to take path points of CARLA so to start from fixed position change in carla_ros_bridge/bridge.py to this: player_start = 771 or any starting position else.

2-turn on autopilot mode on carla and run the following command "python carla_odom.py" and save the points in text file.

3-turn on manual mode on carla and run the following command "roslaunch lane_keeping_pid lane_keeping_pid.launch"

what does the package do:

subscribe to the topic /nav_msgs to get vehicle odometry and after calculating steering angle it  publish it to the topic /ackermann_msgs and publish also constant speed of the vehicle which is 10 meter/sec
