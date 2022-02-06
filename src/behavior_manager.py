#!/usr/bin/env python

## @package behavior_manager
# Here is implemented the state machine that controls the switch between the behaviours of the robot
#
# A finite-state machine (FSM) is a behavior model that consists of a finite number of states. 
# Based on the current state and a given input the machine performs state transitions and produces outputs
# The state machine is implemented using the smach library


import rospy
import smach
import smach_ros
import time
import random

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans


## publisher pub_behavior
#
# the node publishes on the behavior topic using a message of type String.
# the queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
pub_behavior = rospy.Publisher('/behavior', String, queue_size=10)

## subscriber sub_at_home
#
# subscriber to check position
sub_at_home = None



## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
# The robot moves randomly within the Gazebo arena
# - If it receives a "play" command the FSM should go into PLAY state (start_play)
# - If the sleep timer is triggered the FSM should go into SLEEP state (start_sleep)
class Normal_behavior(smach.State):
	## method init
	#
	# This method should:
	# 	- initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['start_sleep','start_play']
		                    )	
		# initialize boolean for checking voice command received or not	
		self.ball_visible = False 
		 # Loop 100Hz
		self.rate = rospy.Rate(1) 

	## method execute
	#
	# - publish "normal" (String) on the topic behavior
	# - check if the ball in the arena is seen by the robot
	# - if the ball is visible:
	#	- goes into PLAY state
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish normal behavior")
		pub_behavior.publish("normal") 

		## check if the ball is visible to the robot
		rospy.Subscriber("/ball_visible", Bool, self.ball_tracking)
		
		# seconds counter
		seconds_counter = 0
		
		# get current time to compute a random interval from the istant
		# in which the robot entered normal state
		start_time = rospy.Time.now()
		
		while not rospy.is_shutdown():
			
			## compute how long the robot stays in Normal state 
			if seconds_counter == 1: 
				current_time = rospy.Time.now()
			seconds_counter += 1
			current_time = rospy.Time.now()
			# compute how long it stays in normal time
			tot_time_given = current_time.secs - start_time.secs	
			if(self.ball_visible):
				## the robot sees the ball it should enter Play behavior
				return 'start_play'
			elif(random.randint(1,100) == 1 and tot_time_given > 30):
				## the robot goes to sleep at random time
				return 'start_sleep'
			self.rate.sleep()				
			
	## method ball_tracking
	#
	# method to check if the ball is visible to the robot or not
	def ball_tracking(self, ball):
		self.ball_visible = ball.data
							
			
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
# The robot should:
#	- reach a predefined location within the arena in Gazebo
#	- stays there for some times 
#	- goes back in NORMAL state
class Sleep_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['stop_sleep']
                            )
		self.at_home = False # initialize boolean variable to check if at home or not
		self.rate = rospy.Rate(20)
		
	## method execute
	#
	# This method should
	# - publish "sleep" (String) on the topic behavior
	# - check if the robot is already at home position or not
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish sleep behavior")
		pub_behavior.publish("sleep") 
		
		# when "sleep" is published on the topic behavior, the node motion should 
		# subscribe to it and send the robot at the home position
		# it now subscribe to Motion to check if it is at home
		# to check if it's already at home position we read the actual position of the robot
		rospy.Subscriber("/at_home", Bool, self.check_if_home_position)
		# we call the callback read_actual_position to save the values in self.position 
		while not rospy.is_shutdown():  
			# check if it is in home position
			if(self.at_home):
				# make it sleep for some time, rospy.sleep()
				rospy.loginfo('NODE BEHAVIOR: Wait some time to wake up')
				rospy.sleep(random.randint(20,35))
				self.at_home = False
				# exit sleep state
				return 'stop_sleep'
		self.rate.sleep
	## method read_actual_position
	#
   	# subscriber to actual_position_robot topic callback, it reads the actual position of the robot
   	def check_if_home_position(self, at_home):
		self.at_home = at_home.data

## class Play_behavior
#
# This class implement the PLAY behavior of the robot pet
# It moves the robot to the predefined (X, Y) location within the map and moves it back to the user.
# The Robot should:
#	- start following the ball
#	- when the ball stops (it means when the robot stops)
#	  it moves the head on the left of 45 degrees, and keep it there for some seconds
#	- then it moves the head to the right, stays there for some seconds
#	- once moved the head keeps tracking the ball

class Play_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['stop_play']
		                    )
		self.ball_visible = False
		self.rate = rospy.Rate(1)  
		self.counter = 0

	## method execute
	#
	# what the robot should do

	def execute(self, userdata):
		# publish behavior "play" on the topic \behavior
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish play behavior")
		pub_behavior.publish("play") 

		# the topic is subscribed to the ball tracking
		rospy.Subscriber("/ball_visible", Bool, self.read_ball_detection)
		
		while not rospy.is_shutdown():
			# check if the ball is visible or not
			if not self.ball_visible:
				rospy.loginfo("NODE BEHAVIOR_MANAGER: searching the ball. Seconds taken:"+str(self.counter)+" seconds")
				# we let the robot look for it for up to 15 seconds 
				if self.counter > 15:
                    			return 'stop_play'
			elif(self.ball_visible):
				self.counter = 0
				
	## method read_ball_detection
	#
	# subscriber callback to find the ball 
	def read_ball_detection(self,ball):
			self.ball_visible = ball.data

## function main
#
# state machine 
def main():
	rospy.init_node("behavior_manager")
	
	# initialization of the sys
	rospy.sleep(2)	

	## Create state machine	
	sm = smach.StateMachine(outcomes=['container_interface'])
	
	## machine container
	with sm:
		## add states to the container,
		smach.StateMachine.add('NORMAL', Normal_behavior(), transitions={'start_sleep':'SLEEP','start_play':'PLAY'})
		smach.StateMachine.add('SLEEP', Sleep_behavior(), transitions={'stop_sleep':'NORMAL'})	
 		smach.StateMachine.add('PLAY', Play_behavior(), transitions={'stop_play':'NORMAL'})	
	
	## Create and start the introspection server for visualization
   	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()
		
	## Wait for ctrl-c to stop the application
    	rospy.spin()
    	sis.stop()


if __name__ == "__main__":
    main()

