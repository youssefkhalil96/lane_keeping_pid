#!/usr/bin/env python
from math import *
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
from vehicle_state.msg import vehicle_pos
from vehicle_state.msg import vehicle_lane_position
import osmnx as ox
from road_info_osm_extract.msg import pointsList
from road_info_osm_extract.msg import points
from road_info_osm_extract.msg import point 

class vehicle_odometry:
	
	def __init__(self,x_init,y_init):
		self._x_init = x_init
		self._y_init = y_init
		self._subscriber = rospy.Subscriber('road_points', pointsList, self.callback)
		self._pub =  rospy.Publisher("vehicle_lane_pos", vehicle_lane_position, queue_size=10, latch=True)
		self._pos = vehicle_lane_position()
		self._road_info = []
		self._path_points = []
		self._road = []
		self._p = []


	def callback(self,data):
		x1 = data.points_list[1].pt[0].x
		y1 = data.points_list[1].pt[0].y
		x2 = data.points_list[1].pt[1].x
		y2 = data.points_list[1].pt[1].y
		x3 = data.points_list[1].pt[2].x
		y3 = data.points_list[1].pt[2].y
		x4 = data.points_list[1].pt[3].x
		y4 = data.points_list[1].pt[3].y
		x5 = data.points_list[1].pt[4].x
		y5 = data.points_list[1].pt[4].y
		road_info = [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x5,y5]]
		print(road_info)
		print("x_start = " + str(self._x_init))
		print("y_start = " + str(self._y_init))

		n_of_roads = len(data.points_list)
		for i in range (0,n_of_roads):
			n_of_points = len(data.points_list[i].pt)
			for j in range(0, n_of_points):
				plt.plot(data.points_list[i].pt[j].x,data.points_list[i].pt[j].y,'r.')


		plt.plot(self._x_init,self._y_init,'*')

		for i in range(0,len(road_info)):
			plt.plot(road_info[i][0],road_info[i][1], 'bo')

		for i in range(0,2):
			self._path_points.append([])
		for i in range (0,2):
			for j in range(0,2):
				self._path_points[i].append(j)
				self._path_points[i][j] = 0
		for i in range(0,2):
			for j in range(0,2):
				self._path_points[i][j] = road_info[i][j]

		a, b, c = self.vehicle_postion(self._x_init,self._y_init)
		self._pos.yaw = a
		self._pos.rd = b
		self._pos.ld = c
		self._pub.publish(self._pos)

		self.vehicle_postion(self._x_init,self._y_init)
		plt.show()




	def vehicle_postion(self,x_init,y_init):

	
		R = sqrt((self._path_points[0][1]-y_init)**2+(self._path_points[0][0]-x_init)**2)
		y = self._path_points[0][1] - y_init
		x = self._path_points[0][0] - x_init
		Rd = 0
		Ld = 0
		# this case will only occur if vehicle arrived to desired started position
		if x == 0 and y == 0:
			yaw_angle_vehicle = math.degrees(math.atan(self._path_points[0][1]-y_init / self._path_points[0][0]-x_init))
			yaw_angle_road = math.degrees(math.atan(self._path_points[1][1]-self._path_points[0][1] / self._path_points[1][0] - self._path_points[0][0]))

			yaw_error = yaw_angle_road - yaw_angle_vehicle
			print("yawerror " + str(yaw_error))
			print("right distance " + str(Rd))
			print("left distance " + str(Ld))


		else:
			slope = float(y / x)
			theta = math.atan(abs(slope))
			delta = R * math.cos(theta)

			if slope > 0:
				Ld = 1.5-delta
				Rd = 1.5+delta
			elif slope < 0:
				Ld = 1.5+delta
				Rd = 1.5-delta 
			# this case means that slope equal zero so the first point of road info is beside the vehicle so we get the slope relative to the second point 
			else:
				R = sqrt((self._path_points[1][1]-y_init)**2+(self._path_points[1][0]-x_init)**2)
				y = self._path_points[1][1] - y_init
				x = self._path_points[1][0] - x_init
				slope = float(y / x)
				theta = math.atan(abs(slope))
				delta = R * math.cos(theta)
				if slope > 0:
					Ld = 1.5-delta
					Rd = 1.5+delta
				elif slope < 0:
					Ld = 1.5+delta
					Rd = 1.5-delta


			yaw_angle_vehicle = math.degrees(math.atan2(y_init , x_init))

			print (yaw_angle_vehicle)
			yaw_angle_road = math.degrees(math.atan2(self._path_points[0][1] , self._path_points[0][0]))

			# the yaw error is very small so I multiply it by 10 power 6
			print(yaw_angle_road)
			yaw_error = (yaw_angle_road - yaw_angle_vehicle)*10**6	

			print("yawerror " + str(yaw_error))
			print("right distance " + str(Rd))
			print("left distance " + str(Ld))


		return yaw_error , Rd , Ld
