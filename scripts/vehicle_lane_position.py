#!/usr/bin/env python

import rospy
from vehicle_lane_odometry import *


if __name__ == '__main__':
	try:

		x_init = 436587
		y_init = 4467419

		position = vehicle_odometry( x_init,y_init)
		rospy.init_node('vehicle_odometry', anonymous=True)
		rospy.spin()

	except rospy.ROSInterruptException:
		print("exception occured")
