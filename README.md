Vehicle lane position:

vehicle_state is a ROS package written in Python that calculate the position of the vehicle with respect to center of the lane.

How to use the package:

1-extract road information that you want to know the position of the vehicle in by clonning "https://github.com/MustafaIsmaill/road_info_osm_extract.git"

2-go to [Nominatim Open Street Maps application](https://nominatim.openstreetmap.org/) and search the area that you want to extract information from, the application should highlight to you the exact area that will be downloaded according to your search string.

3-copy your search string and go to the file "gps_road_estimation/road_info_osm_extract/scripts/extract_road_info.py" and assign the "place_name" variable to your search string.

4-in the file                               "gps_road_estimation/road_info_osm_extract/scripts/extract_road_info.py", rename the "node_name" and change the "publish_rate" variables to match your needs (optional).

5-from the terminal run the following command $ roslaunch road_info_osm_extract road_info_osm_extract.launch

6-subscribe to the "/road_info" topic to receive the road information.

7-assign the starting position of the vehicle by specifying x_init and y_init to the file "vehicle_state/scripts/vehicle_ lane_position.py"

8-from the terminal run the following command $ roslaunch vehicle_state vehicle_state.launch

9-subscribe to the topic "/vehicle_lane_pos"

10-note that the lane width is assumed to be three meters

Package Inputs:
1-x_init
2-y_init

Package Outputs:
1-yaw error
2-distance from rigth lane 
3-distance from left lane
