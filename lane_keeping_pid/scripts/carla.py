#!/usr/bin/env python
import rospy
import math
from math import *
import matplotlib.pyplot as plt
from vehicle_state.msg import vehicle_lane_position
import numpy as np
import time
import matplotlib.pyplot as plt
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry 
from shapely.geometry import Point 
from shapely.geometry import LineString	


class PID:

	def __init__(self , P, I, D):
		self.Kp = P
		self.Ki = I 
		self.Kd = D 
		self.sub = rospy.Subscriber('player_odometry',Odometry,self.callback)
		self.pub =  rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10, latch=True)
		self.ackermann_msg = AckermannDrive()
		self._i = 0
		self.lateral_offset = 0
		self.prev_lateral_offset = 0
		self.current_time = time.time()
		self.last_time = self.current_time
		self.prev_error = 0
		self.yaw_error = 0
		self.Ld = 0
		self.Rd = 0
		self.Velocity_x =10
		self.integral_error = 0
		self.delta_time = 0
		self.error = 0
		self.delta_error = 0
		self.sample_time = 0.1
		self._path_points = []
		self._gps_points = []
		self.yaw_angle_vehicle = 0
		self.delta = 0
		self.i = 0
		self.j = 0
		self.my_tuple = []
		self.angles = []
		self.road_slope = 0

	def callback(self,data):

		self.my_tuple = [data.pose.pose.position.x , data.pose.pose.position.y]
		self.yaw_angle_vehicle = data.pose.pose.orientation.z
		self.vehicle_position()
		self._gps_points.append(self.my_tuple)


	def vehicle_position(self):
		print("****************")
		R = 0
		T =0
		self.j = len(self._gps_points)
		self.j = self.j - 1	

		f = open('road.txt', 'r')
		self._path_points = eval(f.read())

		# storing a list to a file
		outf = open('out','w')
		outf.write(str(self._path_points))



		m = len(self._path_points)
		print("i = " + str(self._i)) 
		print("j = " + str(self.j))
		# print("gps point " + str(self._gps_points))
		print("vehicle position = "+ str(self.my_tuple))
		# plt.plot(self._path_points)
		# plt.plot(self._gps_points,'r')
		# if self._path_points[self._i] == [358.306365967, 2.45851898193]:
		# 	plt.show()

		if self._i < m:
			
			if self.j > 0 :
				dist_vehicle_pose = sqrt((self._path_points[self._i][0] - self.my_tuple[0])**2 + (self._path_points[self._i][1] - self.my_tuple[1])**2)
				dist_prev_vehicle_pose = sqrt((self._path_points[self._i][1] - self._gps_points[self.j][1])**2+(self._path_points[self._i][0] - self._gps_points[self.j][0])**2)
				
				print("R = " + str(dist_vehicle_pose))
				print("T = " + str(dist_prev_vehicle_pose))

				if dist_vehicle_pose > dist_prev_vehicle_pose:

					self._i = self._i + 1

			print("current path point " + str(self._path_points[self._i]))
			

			if self._i == 0:

				R = sqrt((self._path_points[0][1]-self.my_tuple[1])**2+(self._path_points[0][0]-self.my_tuple[0])**2)
				
				y = self._path_points[0][1] - self.my_tuple[1]
				x = self._path_points[0][0] - self.my_tuple[0]
				slope = float(y / x)
				theta = math.atan(abs(slope))

				self.delta = R * math.cos(theta)
			
			else:

				way_point_1 = Point(self._path_points[self._i-1][0],self._path_points[self._i-1][1])
				way_point_2 = Point(self._path_points[self._i][0],self._path_points[self._i][1])
				gps = Point(self.my_tuple[0],self.my_tuple[1])
				points = []
				points.append(way_point_1)
				points.append(way_point_2)
				line = LineString(points)
				self.delta = line.distance(gps)

		

			y = self._path_points[self._i][1] - self.my_tuple[1]
			x = self._path_points[self._i][0] - self.my_tuple[0]
			pos_slope = float(y / x)


			try:

						
				self.road_slope = (self._path_points[self._i+1][1] - self._path_points[self._i][1]) / (self._path_points[self._i+1][0] - self._path_points[self._i][0])
				if self.j == -1:
					yaw_angle_vehicle = math.degrees(math.atan2(self.my_tuple[1] , self.my_tuple[0]))
				else:
					x_v = self.my_tuple[0] - self._gps_points[self.j][0]
					y_v = self.my_tuple[1] - self._gps_points[self.j][1]
					yaw_angle_vehicle = math.atan(y_v/x_v)



				# pos = (x-x1)(y2-y1) - (y-y1)(x2-x1)
				pos = ((self.my_tuple[0] - self._path_points[self._i][0])*(self._path_points[self._i + 1][1] - self._path_points[self._i][1])) - ((self.my_tuple[1] - self._path_points[self._i][1])*(self._path_points[self._i + 1][0] - self._path_points[self._i][0]))
				print(pos)

				if pos > 0:
					self.Rd = 1.5 - self.delta
					self.Ld = 1.5 + self.delta


				elif pos < 0:
					self.Rd = 1.5 + self.delta
					self.Ld = 1.5 - self.delta


				self.yaw_error = abs(abs(math.atan(self.road_slope)) - abs(yaw_angle_vehicle))


				print("yaw angle vehicle = " + str(yaw_angle_vehicle))
				print("road slope = "+ str(math.atan(self.road_slope)))
				
		
			except:
				
				if pos_slope > 0:
					self.Ld = 1.5-self.delta
					self.Rd = 1.5+self.delta
				elif pos_slope < 0:
					self.Ld = 1.5+self.delta
					self.Rd = 1.5-self.delta


				self.yaw_error = abs(0.707 - abs(self.yaw_angle_vehicle))



		self.steering_angle_PID()
		

	def steering_angle_PID(self):
		print("enter steering angle pid")

		if self.i == 0:
			error = self.delta
			delta_error = 0
			self.integral_error = 0
			steering_angle = self.Kp*error + self.Ki*self.integral_error + self.Kd*delta_error

		else:
			
			self.delta_error = (self.delta - self.prev_lateral_offset)+ self.Velocity_x * self.yaw_error 
			

			self.error = self.delta + self.Velocity_x * (self.yaw_error)*0.1
			
			self.integral_error = self.integral_error + self.error 

			steering_angle = self.Kp * self.error + self.Ki*self.integral_error + self.Kd * self.delta_error  


		if self.Ld < self.Rd:
			steering_angle = abs(steering_angle)
			# if steering_angle >10:
			# 	steering_angle = 10
 
	

		elif self.Rd < self.Ld :
			steering_angle = -1*steering_angle
			# if steering_angle < -10:
			# 	steering_angle = -10

		self.prev_lateral_offset = self.delta
		self.i = self.i + 1
		self.ackermann_msg.steering_angle = steering_angle
		self.ackermann_msg.speed = 10
		self.angles.append(steering_angle)
		


		plt.plot(self.angles)  
		plt.xlabel('Iteration Count')
		plt.ylabel('Output')	
		plt.title('PID Controller\n(P=3.5, I=0, D=0.1)')

		print("delta = "+str(self.delta))
		print("Rd = " + str(self.Rd))
		print("Ld = " + str(self.Ld))
		print("yaw error = " + str(self.yaw_error))
		print("steering_angle = " + str(steering_angle))
		self.pub.publish(self.ackermann_msg)















