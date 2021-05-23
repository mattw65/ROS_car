#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
# Import whatever else you think is necessary

# Some useful variable declarations.
angle_range = 0		# sensor angle range of the lidar
desired_trajectory = 0.5	# distance from the wall (left or right - we cad define..but this is defined for right). You should try different values
vel = 10 		# this vel variable is not really used here.
car_length = vel*0.1	# distance (in m) that we project the car forward for correcting the error. You may want to play with this.
error = 0.0

# publish to the topic /team_name/error e.g. /team_alpha/error
pub = rospy.Publisher('/team_golf/error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta

def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here
	# theta is in degrees
	theta_rad = math.radians(theta)
	theta_rad -= math.pi / 2

	angle_range_rad = data.angle_max - data.angle_min

	theta_rad -= data.angle_min # theta_rad is in [angle_min, angle_max]

	index = int(theta_rad / data.angle_increment)

	max_index = int(angle_range_rad / data.angle_increment)
	min_index = 0

	if index > max_index or index < min_index:
		return -1

	range = data.ranges[index]

	if math.isinf(range) or range > data.range_max:
		return data.range_max

	if index < data.range_min:
		return data.range_min

	if math.isnan(range):
		return -1

	return range

def callback(data):
	# choose left or right wall to follow
	left = getRange(data, 180)
	right = getRange(data, 0)
	forward = getRange(data, 90)
	# print 'forward: %f' % forward

	theta = 45

	is_left = False
	# if left > right:
	# 	is_left = False
	# print 'is_left: %r' %is_left
	if is_left:
		theta2 = 180
		theta1 = theta2 - theta
	else:
		theta2 = 0
		theta1 = theta2 + theta
	
	a = getRange(data,theta1)
	b = getRange(data,theta2)	# Note that the 0 implies a horizontal ray..the actual angle for the LIDAR may be 30 degrees and not 0.
	swing = math.radians(theta)

	## Your code goes here to compute alpha, AB, and CD..and finally the error.
	alpha = math.atan((a*math.cos(swing) - b)/(a*math.sin(swing)))
	AB = b*math.cos(alpha)
	AC = car_length
	CD = AB + AC*math.sin(alpha)
	if is_left:
		y = -(desired_trajectory - CD)
	else:
		y = desired_trajectory - CD
	error = -(y+AC*math.sin(alpha))
	# error = -y+AC*math.sin(alpha)
	# print 'error: %f' % error
	# print 'alpha: %f' % alpha
	# error = 0
	if forward < 1.7:
		if left < 1.7:
			error = -20
		else:
			error = 20
	# 	if right > 2:
	# 		error = -20
	# 	elif left > 2:
	# 		error = 20
	# 	# else:
	# 	# 	error = 20
	else:
		error = 0
	## END

	msg = pid_input()
	msg.pid_error = error		# this is the error that you want to send to the PID for steering correction.
	msg.pid_vel = vel		# velocity error is only provided as an extra credit field.
	pub.publish(msg)


if __name__ == '__main__':
	print("Laser node started")
	# name your node team_name_dist_finder e.g. team_alpha_dist_finder
	rospy.init_node('team_golf_dist_finder',anonymous = True)

	# subscribe to the correct /team_name/scan topic for your team's car..
	rospy.Subscriber("/team_golf/scan",LaserScan,callback)

	rospy.spin()
