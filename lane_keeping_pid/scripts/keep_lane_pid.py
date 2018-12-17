#!/usr/bin/env python

import rospy
from carla import *

if __name__ == '__main__':
	try:
		rospy.init_node('PID', anonymous=True)
		steering_control = PID(4,0,0.1)

		rospy.spin()

	except rospy.ROSInterruptException:
		print("exception occured")
