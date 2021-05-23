#!/usr/bin/env python

import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
import math

kp = 14.0
kd = 0.09
servo_offset = 0	# zero correction offset in case servo is misaligned. 
prev_error = 0.0 
vel_input = 25.0	# arbitrarily initialized. 25 is not a special value. This code can accept input desired velocity from the user.


pub = rospy.Publisher('/team_golf/multiplexer/command', AckermannDrive, queue_size=10)
# setup a publisher to publish to the /car_x/offboard/command topic for your racecar.


def control(data):
	global prev_error
	global vel_input
	global kp
	global kd

	## Your code goes here
	# 1. Scale the error 
	# 2. Apply the PID equation on error to compute steering
	# 3. Make sure the steering value is within bounds
 	error = data.pid_error
	# print 'error: %f' % error
	angle = kp*error+kd*(error - prev_error)

	vel = vel_input
	# vel = vel_input - error
	# if vel < 1:
	# 	vel = 1

	if angle > 100:
		angle = 100
	if angle < -100:
		angle = -100

	max_steering_angle = math.pi / 2

	angle = angle / 100 * max_steering_angle
	
	# print 'angle: %f' % angle

	prev_error=error

	# angle = max_steering_angle
	## END

	msg = AckermannDrive()
	msg.speed = vel
	msg.steering_angle = angle
	msg.steering_angle_velocity = 0
	msg.acceleration = 0
	msg.jerk = 0
	pub.publish(msg)


if __name__ == '__main__':
	global kp
	global kd
	global vel_input
	print("Listening to error for PID")
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	vel_input = input("Enter Velocity: ")
	rospy.init_node('team_golf_pid_controller', anonymous=True)
	rospy.Subscriber("/team_golf/error", pid_input, control)
	msg = AckermannDrive()
	msg.speed = 0
	pub.publish(msg)
	rospy.spin()
