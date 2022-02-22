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
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from assignment2.msg import Ball_state
from assignment2.msg import HeadState

VERBOSE = False


## class track_ball
#
# used to track the ball moving in the environment
# it makes use of cv2
class track_ball:
	## method __init__
	# 
	# initialization of the class
	def __init__(self):
		rospy.init_node('ballDetection', anonymous = True)

		## Publishers
		# publisher to sent processed and compressed images
		self.pubImage = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
		self.pubBallState = rospy.Publisher("ball_state",Ball_state,queue_size=1)
		# publish robot velocity
		self.pubVel = rospy.Publisher("cmd_vel",Twist, queue_size=1)

		## Subscribers
		# subscriber to camera
		self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
		# subscriber to head_state to know when the head has stopped moving
		self.head_state_sub = rospy.Subscriber("head_state",HeadState, self.get_head_info, queue_size = 1)

		# Variable to check if the head is in the upright position 
		self.headState = True
		# check the robot has reached the ball
		self.atBall = False 

	## method get_head_info
	#
    	def get_head_info(self, data): 
        	self.headState = data.HeadMotionStop

	##  method callback
	#
	# callback function of topic subscribed
	# we use it to read information on the detection and the converted image
	def callback(self, ros_data):
		
		if VERBOSE:
			print('*** OPENCV TRACK: Image received. Type: "%s"' % ros_data.format)
			
		## openCV algorithm
		
		## cv2 conversion
		np_arr = np.fromstring(ros_data.data, np.uint8)
		# the function imdecode compresses the image and stores it in memory buffer         			
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
		# colour thresholds
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
		center = None
			
		# only proceed if at least one contour was found
		if len(cnts) > 0:
		    	# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			# get circle which completely converts the object with minimum area.
			((x, y), radius) = cv2.minEnclosingCircle(c)
			# M dictionary of all moments values calculated 
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame using circle()
				# then update the list of tracked points
				cv2.circle(image_np, (int(x), int(y)), int(radius),
			    		(0, 255, 255), 2)
				cv2.circle(image_np, center, 5, (0, 0, 255), -1)
				
				# ROS msg
				msg = Ball_state()
				# ball visible
				msg.ballVisible = True			
				# at ball
				msg.atBall = self.atBall
			
				# we need to publish if the ball is visible or not
            			self.pubBallState.publish(msg)

				
				# check if the robot has reahced the object or not
				# head not moving
				if self.headState == True:
					rospy.loginfo("*** OPENCV TRACK: ball is visible")
					rospy.loginfo("*** OPENCV TRACK: robot start moving")
					vel = Twist()
					# compute velocities
					# 400 is the center of the image 
		        		vel.angular.z = -0.002*(center[0]-400)
					# 150 is the desired distance from the object 
		        		vel.linear.x = -0.01*(radius-150)
		        		self.pubVel.publish(vel)
					self.atBall = False


					if(radius >= 148) and abs(center[0]-400) < 5:
						# ball has been reached
						rospy.loginfo("*** OPENCV TRACK: robot has reached the ball")
						self.headState = False
						self.atBall = True
						msg.atBall = self.atBall
						self.pubBallState.publish(msg)
	
		else:
			msg = Ball_state()
			msg.ballVisible = False
			self.pubBallState.publish(msg)

		# update the points queue
		# pts.appendleft(center)
		cv2.imshow('window', image_np)
		cv2.waitKey(2)

		# self.subscriber.unregister()	
			

## function main
#
# 
def main(args):
	# init trackin node
	inode = track_ball()

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print ("Shutting down ROS Image feature detector module")
    	cv2.destroyAllWindows()

if __name__ == '__main__':	
	main(sys.argv)

