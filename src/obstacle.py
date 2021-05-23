#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
# Import whatever else you think is necessary

# Some useful variable declarations.
car_radius = 0.25    					# the radius of the circle that encompasses the entire car
fov_angle = 180							# field-of-view angle in degrees
fov_angle_rad = math.radians(fov_angle)	# field-of-view angle in radians

min_edge_disparity = 0.3	# minimum difference between to scans to be considered an edge

min_gap_width = 3	# minimum number of lidar scans for a gap
min_gap_depths = [2, 1]	# minimum distance from car for a gap

# publish to the topic /team_name/multiplexer/command e.g. /team_alpha/multiplexer/command
pub = rospy.Publisher('/team_golf/multiplexer/command', AckermannDrive, queue_size=10)

# Input:	data: Lidar scan data
# Output:	an array of lidar scans for the field-of-view
def get_forward_fov(data):
	scans = []
	# print "min: %f" % data.angle_min
	# print "max: %f" % data.angle_max
	start_index = int((-fov_angle_rad / 2 - data.angle_min) / data.angle_increment)
	end_index = int(start_index + fov_angle_rad / data.angle_increment)
	# print "start_index: %f" % start_index
	# print "end_index: %f" % end_index
	# print "angle_increment: %f" % data.angle_increment
	scans = list(data.ranges[start_index:end_index+1])
	# print "length of scans: %i" % len(scans)
	return scans

# Input:	scans:	an array of lidar scans for the field-of-view
# Output:	an array of indexes where edges where found (the closer value of the edge)
def get_edges(scans):
	edges = []

	# loop through scans and add angle if there is a large discrepancy in distance between it and the next angle
	for i, curr_scan in enumerate(scans):
		if i == len(scans) - 1:	# the last element of scans
			break

		next_scan = scans[i+1]
		if math.fabs(curr_scan-next_scan) > min_edge_disparity:
			edges.append(i if curr_scan < next_scan else i+1)

	# print "num_edges: %u" % len(edges)

	return edges

# Input:	scans:	an array of lidar scans for the field-of-view
# 			edges:	an array of indexes where edges where found (the closer value of the edge)
# 			data:	Lidar scan data
# Output:	modified scans array
def inflate_edges(scans, edges, data):
	for edge in edges:
		d = scans[edge]
		r = car_radius
		theta = math.atan(r/d)
		inflation_width = int(theta / data.angle_increment)
		#print "inflation width: %f" % inflation_width
		for i in range(edge-inflation_width, edge+inflation_width+1):
			if i < 0 or i >= len(scans):
				continue
			if scans[i] > d:
				scans[i] = d

	return scans

# Input:	scans:	an array of lidar scans for the field-of-view
# Output:	gaps:	an array of gaps found in scans, where each gap is a tuple of (index, width, depth)
def get_gaps(scans):
	gaps = []
	for min_gap_depth in min_gap_depths:
		gaps = []

		curr_gap_index = 0
		curr_gap_width = 0
		curr_gap_depth = 0

		in_gap = False

		for i in range(len(scans)):
			if in_gap:
				if scans[i] > min_gap_depth:
					curr_gap_width += 1
					if scans[i] > curr_gap_depth:
						curr_gap_index = i
						curr_gap_depth = scans[i]
				else:
					in_gap = False
					# calculate gap
					if curr_gap_width >= min_gap_width:
						# valid gap
						gaps.append((curr_gap_index, curr_gap_width, curr_gap_depth))
			else:
				if scans[i] > min_gap_depth:
					# print "scans[i]: %f" % scans[i]
					curr_gap_index = i
					curr_gap_width = 1
					curr_gap_depth = scans[i]
					in_gap = True
		if in_gap:
			if curr_gap_width >= min_gap_width:
				# valid gap
				gaps.append((curr_gap_index, curr_gap_width, curr_gap_depth))

		if len(gaps) > 0:	# at least 1 gap was found 
			break
	print "gaps found: %u" % len(gaps)
	return (gaps, min_gap_depth)

# Input:	gaps:	an array of gaps found in scans, where each gap is a tuple of (index, width, depth)
# 			scans:	an array of lidar scans for the field-of-view
# Output:	index:	index in scans array of best gap
def choose_gap(gaps, scans, min_gap_depth):
	best_gap = (-1, 0, 0)

	mid_index = len(scans) // 2

	for gap in gaps:
		curr_gap_index, curr_gap_width, curr_gap_depth = gap
		best_gap_index, best_gap_width, best_gap_depth = best_gap

		if min_gap_depth == min_gap_depths[0]:
			if abs(curr_gap_index - mid_index) < abs(best_gap_index - mid_index):
				best_gap = gap
		else:
			if curr_gap_depth > best_gap_depth:
				best_gap = gap

	return best_gap[0]

# Input:	best_dist:	an array of lidar scans for the field-of-view
# 			best_angle:	index in scans array of best gap
# Output:	vel:		steering velocity
# 			angle:		steering angle
def steering(best_dist, best_angle):
	# print "best_dist: %f" % best_dist
	angle = best_angle - math.radians(90)

	print 'steering angle: %f' % (angle)
	print 'best dist: %f' % (best_dist)
	vel = 1 * (best_dist/10)**1.2 * ((math.radians(180) - math.fabs(angle))/math.pi)**2
	print "vel: %f" % vel
	if best_dist < 0:
		return (0, 0)
	elif math.isinf(best_dist) or vel > 2:
		vel = 2

	return (vel, 1.1*angle)

def callback(data):
	## Forward Field of View
	scans = get_forward_fov(data)

	## Disparity Detector
	edges = get_edges(scans)

	## Inflation
	scans = inflate_edges(scans, edges, data)

	## Pick the best Gap
	gaps, min_gap_depth = get_gaps(scans)
	best_gap_index = choose_gap(gaps, scans, min_gap_depth)

	# convert index to distance and angle
	if best_gap_index < 0:
		best_gap_dist = -1
		best_gap_angle = -1
	else:
		best_gap_dist = scans[best_gap_index]
		best_gap_angle = best_gap_index*data.angle_increment

	## Steering
	vel, angle = steering(best_gap_dist, best_gap_angle)

	## END

	msg = AckermannDrive()
	msg.speed = vel
	msg.steering_angle = angle
	msg.steering_angle_velocity = 0
	msg.acceleration = 0
	msg.jerk = 0
	pub.publish(msg)


if __name__ == '__main__':
	print("Obstacle node started")
	# name your node team_name_dist_finder e.g. team_alpha_dist_finder
	rospy.init_node('team_golf_obstacle',anonymous = True)

	# subscribe to the correct /team_name/scan topic for your team's car..
	rospy.Subscriber("/team_golf/scan",LaserScan,callback)

	msg = AckermannDrive()
	msg.speed = 0
	pub.publish(msg)

	rospy.spin()
