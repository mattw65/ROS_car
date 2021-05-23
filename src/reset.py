#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

# publish to the topic /team_name/multiplexer/command e.g. /team_alpha/multiplexer/command
pub = rospy.Publisher('/team_golf/multiplexer/command', AckermannDrive, queue_size=10)

if __name__ == '__main__':
	# name your node team_name_dist_finder e.g. team_alpha_dist_finder
	rospy.init_node('team_golf_reset',anonymous = True)

	msg = AckermannDrive()
	msg.speed = 0
	pub.publish(msg)

	print("Reset car!")
