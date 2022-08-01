#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import pdb

# Vehicle parameters
ANGLE_RANGE = 360          # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 0.2    # Distance threshold before collision (m)
VELOCITY = 0.05              # Maximum Velocity of the vehicle
TIME_THRESHOLD = 0.5        # Time threshold before collision (s)
STEERING_ANGLE = 0          # Steering angle is uncontrolled

# P-Controller Parameters
kp_dist = 0.75 # initialize
kp_ttc = 0.5 # initialize

dist_error = 0.0
time_error = 0.0

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def dist_control(distance):
	global kp_dist
	global VELOCITY
	global DISTANCE_THRESHOLD 
	global STEERING_ANGLE
	
	# Calculate Distance to Collision Error
	if distance > DISTANCE_THRESHOLD:
	   if distance <= 0.4:
	      dist_error = distance - DISTANCE_THRESHOLD # only need distance error
	      velocity = kp_dist * dist_error
	   else:
	      velocity = VELOCITY
	else:
	   velocity = 0.0

	print("Distance before collision is = ", distance)
	print("Vehicle velocity = ", velocity)

	msg = Twist()
	msg.linear.x = velocity
	msg.angular.z = STEERING_ANGLE
	pub.publish(msg)

def TTC_control(distance):
	global kp_ttc
	global TIME_THRESHOLD
	global VELOCITY
	global STEERING_ANGLE

	# Calculate Time To Collision Error
	time = distance / VELOCITY # time to collision
	if time > TIME_THRESHOLD:
	   if time == np.float('inf'):
	      velocity = VELOCITY
	   else:
	      ttc_error = time - TIME_THRESHOLD
	      dist_error = distance - DISTANCE_THRESHOLD
	      velocity = VELOCITY = kp_ttc * (dist_error / ttc_error)
	else:
	   velocity = 0.0
		
	print("Time to collision in seconds is = ", time)
	print("Vehicle velocity = ", velocity)

	msg = Twist()
	msg.linear.x = velocity
	msg.angular.z = STEERING_ANGLE
	pub.publish(msg)

def get_index(angle, data):
	global ANGLE_RANGE

	# For a given angle, return the corresponding index for the data.ranges array
	ilen = len(data.ranges)
	mid = angle / 2.0
	ipd = ilen / ANGLE_RANGE # index per degree
	lwr_bound = int(ipd * mid)
	upr_bound = int(ilen - lwr_bound)
	return np.array([i for i in range(ilen) if i <= lwr_bound or i >= upr_bound])
	
# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data): 
	global ANGLE_RANGE
	
	angle_front = 20   # Range of angle in the front of the vehicle we want to observe
	avg_dist = 0
	
	# Get the corresponding list of indices for given range of angles
	index_front = get_index(angle_front, data)

	# Find the avg range distance
	ranges = np.array(data.ranges)
	avg_dist = np.average(ranges[index_front])
	
	print("Average Distance = ", avg_dist)

	return avg_dist

def callback(data):

	# Complete the Callback. Get the distance and input it into the controller
	distance = get_distance(data)
	
	# Euclidean distance control
	dist_control(distance)

if __name__ == '__main__':
	print("AEB started")
	rospy.init_node('aeb', anonymous = True)
	rospy.Subscriber("/scan", LaserScan,callback)
	rospy.spin()
