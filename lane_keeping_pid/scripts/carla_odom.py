#!/usr/bin/env python

from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import rospy
import roslib
import tf
import numpy as np
import matplotlib.pyplot as plt
import time

show_animation = False

global_path = Path()
path_received = False
lookahead_distance = 8



if __name__ == '__main__':

    text_file = open("road.txt", "a")
    rospy.init_node('carlaodom')
    listener = tf.TransformListener()

    prev_trans = [0, 0]
    string = ''

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if np.sqrt((prev_trans[0] - trans[0])**2 + (prev_trans[1] - trans[1])**2) > 0.3:
            prev_trans[0] = trans[0]
            prev_trans[1] = trans[1]
            string = '[' + str(trans[0]) + ', ' + str(trans[1]) + '], '
            text_file.write(string)


    print(string)
    time.sleep(0.5)





#-((5*p0x - 5*p1x)*(20*p0y - 40*p1y + 20*p2y) - (5*p0y - 5*p1y)*(20*p0x - 40*p1x - (20*ord)/pend + (20*p2y)/pend))/((5*p0x - 5*p1x)^2 + (5*p0y - 5*p1y)^2)^(3/2)
