#!/usr/bin/env python

## @package human_simulator
#
# Human interactions with the ball
# The human can:
# 	1) move the ball around
#		- give a command position to the ball
#		- wait for a number of seconds
#		- give another command to the position of the ball
#	2) make the ball disappear
#		- it sends a position with negative z

from __future__ import print_function
import rospy 
import random
import time 
import actionlib
import actionlib_tutorials.msg
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from assignment2.msg import Planning_ballAction, Planning_ballGoal, Planning_ballResult, Planning_ballFeedback



## main function
#
# initialize the action client and move the ball
def main():
	#init node
	rospy.init_node("human_simulator")	
		
	# counter to make the ball disappear
	ball_disappear = 0
	
	# init client
	client = actionlib.SimpleActionClient('reaching_goal', Planning_ballAction)
   	client.wait_for_server()
	
	# init ball goal position 
     	goal = Planning_ballGoal()

	time.sleep(5)
     
	while True:
		# make ball disappear
	 	if ball_disappear == 3:
			rospy.loginfo('*** HUMAN SIMULATOR: make ball disappear')
			goal.target_pose.pose.position.z = -1
			client.send_goal(goal)
         		client.wait_for_result()
			time.sleep(10)
   			ball_disappear = 0
			
		# choose random position
		goal.target_pose.pose.position.x = random.randint(-5,5)
         	goal.target_pose.pose.position.y = random.randint(-5,5)
	 	goal.target_pose.pose.position.z = 0.5
		
		# send goal position
	 	client.send_goal(goal)
         	client.wait_for_result()
	 	
		rate = rospy.Rate(1)
		
		# wait for some time
		randi = random.randint(3,8)
		for i in range(randi):
			rate.sleep()

		# increment counter
	 	ball_disappear = ball_disappear + 1

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
