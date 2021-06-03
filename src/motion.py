#!/usr/bin/env python

## @package motion
#
# It moves the robot within the map respecting the behavior

import rospy
import time
import random

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Int32 # needed for publishing integers
from first_assignment.msg import IntArray # needed to publish/subscribe [x,y] describing the position of the robot
from map2Dclass import Map2D # class to simulate map of the environment


## global variables
behaviour = None # used to check current behavior
at_home = False # used to check if the robot is at home position or not
goal = None # goal position to reach [x,y]

random_time = 0.5 # NB remember to get param from launch file

## object Map2D, we need it to access the values describing the map 2D within which the robot is moving 
map_2D = Map2D()

## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	#rospy.loginfo('NODE MOTION: Executing callback behavior')
	global behaviour 
	behaviour = data.data

## function update_position
#
# update actual position of the robot with the given one
def update_position(x,y):
	map_2D.x_actual=x # update with the given x
	map_2D.y_actual=y # update with the given y

## function move_random
#
# the robot moves randomly when in the NORMAL state
def move_normal():
	rospy.loginfo("NODE MOTION: Execute function to move in normal mode and reach a random position")
	## get random position
	randX = random.randint(0,map_2D.x_max) 
	randY = random.randint(0,map_2D.y_max) 
	randPos = [randX,randY]
	## update actual position
	update_position(randPos[0],randPos[1])
	## wait random time to simulate the robot has moved and reached position
	rospy.loginfo('NODE MOTION: robot reached random position')
	rospy.sleep(random_time*random.randint(2,15))
	

## function move_reach_user
#
# the robot reached the user when in the PLAY state
def move_reach_user():
	## get random position	
	rospy.loginfo('NODE MOTION: play state - robot at user')
	rospy.sleep(random_time*random.randint(2,12))
	update_position(map_2D.x_home,map_2D.y_home)


## function move_sleep_position
#
# movement in the SLEEP state
def move_sleep_position():
	
	global at_home # used to check if the robot is at home position or not
	## go to the home position
	if not at_home:
		rospy.loginfo("NODE MOTION: move into sleep position")
	        ## wait random time to simulate reaching the point
		rospy.loginfo('NODE MOTION: The robot is asleep at home position')
	        rospy.sleep(random_time*random.randint(1,4))
	        update_position(map_2D.x_home,map_2D.y_home)
	        at_home = True

## function callback_get_position
#
# subscriber callback position
def callback_get_position(position):
    global goal
    goal = position.data


## function reach_goal
#
# move to the goal position given by the user pointing gesture
def reach_goal():    
	## go to the pointed position 
	rospy.sleep(random_time*random.randint(3,10))
	rospy.loginfo('NODE MOTION: play state - goal position reached')
	# call function update_position(x,y)
		# map_2D.x_actual=x
		# map_2D.y_actual=y
	update_position(goal[0],goal[1])

## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	
	global at_home # boolean to check if robot at home or not
	global goal # goal position to reach 
	
	# temporary store actual position we need to check if it is changing or not (see end of this code)
	x_temp = map_2D.x_actual
	y_temp = map_2D.y_actual
	
	## subscriber
	rospy.loginfo('NODE MOTION: Subscriber to /behavior topic and /pointing_gesture')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)		
	rospy.Subscriber("/pointing_gesture",IntArray, callback_get_position)

	## publisher of actual position of the robot
	#
	# publish actual position on the topic actual_position_robot which is subscibed by behavior_manager in Sleep state
	pub_actual = rospy.Publisher("/actual_position_robot",IntArray,queue_size=10)

	rate = rospy.Rate(110)
	# we impose that the initial position corresponds to home position 
	map_2D.x_actual= map_2D.x_home 
	map_2D.y_actual= map_2D.y_home 
	
	## pub initial position
    	pub_actual.publish([map_2D.x_actual,map_2D.y_actual])

	## move according to the behaviour
	while not rospy.is_shutdown():
		if(behaviour == "normal"):
			#print('NODE MOTION: behaviour ', behaviour)
			at_home = False 
			## robot is moving randomly, call function move_normal()
			move_normal()
			## ignore pointing command
            		if not goal == None:
                		goal = None
		else:			
			# check robot behavior
			if(behaviour == "sleep"):
				#print('NODE MOTION: behaviour ', behaviour)
				## the robot moves to predefined location
				# call function move_sleep_position()
				move_sleep_position()
				## we initialize goal al null to ignore the goal position command from the user 
		    		if not goal == None:
		        		goal = None
			else:
				if(behaviour == "play"):
					print('NODE MOTION: behaviour ', behaviour)
					## the robot goes to the user location
					## we first check it is not there already
					if not ((map_2D.x_actual,map_2D.y_actual) == (map_2D.x_user,map_2D.y_user)):
						# if not at user position, the robot should reach him
						## call function move_reach_user() to reach the user posotion
						move_reach_user() 
					
					if not goal == None:
						## waits for a pointing gesture
						## goes in the pointed location
						reach_goal()
						## position is now reached, we don't have a goal anymore
						goal = None
				
	## for each behavior
	if not (behaviour == None):
		## if the position is not changed we don't update the actual position
		if not ((map_2D.x_actual == x_temp) & ((map_2D.y_actual == y_temp))):
                	pub_actual.publish([map_2D.x_actual,map_2D.y_actual])		

	rate.sleep()
	

if __name__ == "__main__":
    main()
