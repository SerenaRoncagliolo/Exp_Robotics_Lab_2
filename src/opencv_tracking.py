#!/usr/bin/env python

## @package opencv_tracking
#
# it make uses of openCV libraries to track the ball moving within the map so that
# the robot can follow it

import sys
import time

import numpy as np
from scipy.ndimage import filters

import imutils # image processing functions easier with OpenCV and both Python 2.7 and Python 3. 

# OpenCV 
import cv2

# Ros libraries
import roslib
import rospy

# import ROS Messages
from sensor_msgs.msg import CompressedImage, JointState 
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float64
from random import randint
import math 

VERBOSE = False

publisherHeadPos = None

## class track_ball
#
# used to track the ball moving in the environment
# it makes use of cv2
class track_ball:
	## method __init__
	# 
	# initialization of the class
	def __init__(self):
		# we need the following
		#	- ball dimensions described by its center and radius
		#	- boolean to check if ball if visible or not
		#	- boolean to check if the ball is moving or not
		# 	- boolean to check if the robot is close to the ball or not
		#	- robot behavior
		self.center = None
        self.radius = None
		self.ball_visible = False
       	self.ball_stop = False
        self.near_ball = False
        self.behaviour = None

		# publish robot velocity
		self.vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
	    # publish if ball detected
		self.publisherBall = rospy.Publisher("/ball_visible", Bool, queue_size=1)
		
		# subscribe to camera
		self.cam_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)
	
		# subscriber to current behaviour
        rospy.Subscriber("/behavior", String, self.get_behaviour)
	## method get_behavior
	#
	# subscriber callback to the behavior topi
	def get_behavior(self,state):
		self.behavior = state.data	

	## method follow_ball
	#
	# method to set the robot velocity so that it can follow the ball
	def follow_ball(self):
		if self.ball_stop:
			# if the ball is not moving so should the robot
			# we set all velocities to zero
			twist_msg = Twist()
			twist_msg.angular.z = 0.0
            		twist_msg.linear.x = 0.0
           		self.vel_pub.publish(twist_msg)
		else:
			# if the ball is visible, the robot should move towards it
			# and follow it while moving
			if self.ball_visible:
				if self.near_ball:
					# if near enough -> it follows it 
					twist_msg = Twist()
					twist_msg.angular.z = 0.003*(self.center[0] - 400)
					twist_msg.linear.x = -0.01*(self.radius - 100)
					self.vel_pub.publish(twist_msg) 
				else:
					# if the robot not near enough -> move to the ball
					# 					
					twist_msg = Twist()
                    twist_msg.linear.x = 0.8
                    self.vel_pub.publish(twist_msg)
			# if the ball is not visible the robot should see it
			elif not self.ball_visible:
				twist_msg = Twist()
				twist_msg.angular.z = 0.9 # to rotate
				self.vel_pub.publish(twist_msg)
	##  method callback_sub
	#
	# callback function of topic subscribed
	# we use it to read information on the detection and the converted image
	def callback(self, ros_data):
		if not self.ball_stop:
			if VERBOSE:
				print('Image received. Type: "%s"' % ros_data.format)
			angular_z = None
			linear_x = None
			## cv2 conversion
			np_arr = np.fromstring(ros_data.data, np.uint8)
			# the function imdecode compresses the image and stores it in memory buffer         			
			image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
			# thresholds
			greenLower = (50, 50, 20)
            greenUpper = (70, 255, 255)
			# Blurs image using a Gaussian filter.
			blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
			# convert image from one color space to another.
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			# threshold operations            
			mask = cv2.inRange(hsv, greenLower, greenUpper)
			# perform erosion on the image
            mask = cv2.erode(mask, None, iterations=2)
			# do opposite
            mask = cv2.dilate(mask, None, iterations=2)

            # Contours -> curve joining all the continuous points 
			# having same color or intensity. 
			# The contours are a useful tool for shape analysis and 
			# object detection and recognition.
			# For better accuracy apply threshold
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            self.center = None
            # only proceed if at least one contour was found
			if len(cnts) > 0:
            	# find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
				# get circle which completely covers the object with minimum area.
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                self.radius = radius
				# M dictionary of all moments values calculated 
                M = cv2.moments(c)
                self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				# ball is visible
				self.ball_visible = True

				# only proceed if the radius meets a minimum size
				if radius > 10:
				    # draw the circle and centroid on the frame using circle()
				    # then update the list of tracked points
				    cv2.circle(image_np, (int(x), int(y)), int(radius),
				    		(0, 255, 255), 2)
				    cv2.circle(image_np, self.center, 5, (0, 0, 255), -1)
					# robot is near the ball
				    self.near_ball = True
				else:
					self.near_ball = False

			else:
				self.ball_visible = False
			
			# we need to publish if the ball is visible or not
            self.publisherBall.publish(self.ball_visible)

            # update the points queue
            cv2.imshow('window', image_np)
            cv2.waitKey(2)

            # if behaviour is play, follow the ball
            if self.behaviour == "play":
                if self.ball_detected and self.near_ball:
                    angular_z = 0.003*(self.center[0] - 400)
                    linear_x = -0.01*(self.radius - 100)
                    # if the ball is still, move the head
                    if abs(angular_z) < 0.04 and abs(linear_x) < 0.04 :
                        # signal that the ball has stopped
                        self.ball_stopped = True
                # follow the ball
                self.follow_ball()

## function move_head
#
# function which moves the head of the robot when the ball stops
def move_head():
	rospy.loginfo("NODE BALL TRACKING: ball has stopped")
	# define angle to move head
    angleStart = math.pi/4

	# move the head in one direction 45 deg
	# wait random time
	angleHead = Float64()
	angleHead.data = angleStart
	pubHeadPos.publish(angleHead)
	rospy.sleep(randint(3,6)) # wait some seconds

	# move the head in the opposite direction 45 deg
	angleHead = Float64()
	angleHead.data = -angleStart
	pubHeadPos.publish(angleHead)
	rospy.sleep(randint(3,6)) # wait some seconds
	
	# move the head in default direction 
	angleHead = Float64()
	angleHead.data = 0
	pubHeadPos.publish(angleHead)

## function main
#
# 
def main(args):
	# init trackin node
	rospy.init_node('opencv_tracking', anonymous = True)
	
	global publisherHeadPos

	rate = rospy.Rate(100)
	
	publisherHeadPos = rospy.Publisher("/robot/joint_position_controller/command", Float64, queue_size=1)

	# init class
	trackBall = track_ball()
	
	while not rospy.is_shutdown():
		# when the ball stops, the robot stops and moves its head
		if trackBall.ball_stop:
			# move robot head
			move_head()
			rospy.sleep(2)
			trackBall.ball_stop = False
			rospy.loginfo("NODE OPCV_TRACK: tracking ball")
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print ("Shutting down ROS Image feature detector module")
    	cv2.destroyAllWindows()

if __name__ == '__main__':	
	main(sys.argv)

