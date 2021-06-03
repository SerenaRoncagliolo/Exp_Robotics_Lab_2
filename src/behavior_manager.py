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
from std_msgs.msg import Int32 # needed for publishing integers
from first_assignment.msg import IntArray # needed to publish/subscribe [x,y] describing the position of the robot
from map2Dclass import Map2D # class to simulate map of the environment

## global variables
random_time = 0.5 

## object Map2D, we need it to access the values describing the map 2D within which the robot is moving 
map_2D = Map2D()

## publisher pub_behavior
#
# the node publishes on the behavior topic using a message of type String.
# the queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
pub_behavior = rospy.Publisher('/behavior', String, queue_size=10)



## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
# The robot moves randomly within the map 
# - If it receives a "play" command the FSM should go into PLAY state 
# - If the sleep timer is triggered the FSM should go into SLEEP state
class Normal_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['start_sleep','start_play']
		                    )	
		# initialize boolean for checking voice command received or not	
		self.play_command_received = False 
		 # Loop 100Hz
		self.rate = rospy.Rate(1) 
		rospy.Subscriber("/voice_command", String, self.get_command)
	## method execute
	#
	# - publish "normal" (String) on the topic behavior
	# - check if a voice command is received from the user
	# - if the command is received:
	#	- goes into PLAY state
	# - else
	#	- trigger sleeping timer at random time
	#	- goes into SLEEP state
	def execute(self, userdata):
		pub_behavior.publish("normal") 
		# self.counter = random.randint(1,2) 
		## check if the user command is received, subscribe to the topic voice_command on which voice_command.py publishes "play"
		while not rospy.is_shutdown():
			# if command play received
			# check if boolean play_command_received is True
			## wait random time
			rospy.sleep(random_time*random.randint(5,30))
			if(random.randint(1,6) == 5):
				return 'start_sleep' 			
			else:
				if(self.play_command_received):
					self.play_command_received = False
					return 'start_play'
			self.rate.sleep()
	## method get_command
	#
	# method to get the voice command
	def get_command(self, command):
		if(command.data == "play"):
			self.play_command_received = True

							
			
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot should:
#	- reach a predefined location
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
		self.rate = rospy.Rate(1)
		self.position = [-1,-1] # initialise position
	## method execute
	#
	# - publish "sleep" (String) on the topic behavior
	# - publish actual position (IntArray [x,y]) of the robot, so that the motion node can check if the robot is already at home or not
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish sleep behavior")
		pub_behavior.publish("sleep") 
		# when "sleep" is published on the topic behavior, the node motion should subscribe to it 
		# and send the robot at the home position
		# to check if it's already at home position we read the actual position of the robot
		rospy.Subscriber("/actual_position_robot", IntArray, self.read_actual_position)
		# we call the callback read_actual_position to save the values in self.position 
		while not rospy.is_shutdown():  
			# check it at home position [xhome, yhome]
			#if(self.position == (map_2D.x_home,map_2D.y_home)):
			## it should sleep for some time
			rospy.loginfo('NODE BEHAVIOR: Wait some time to wake up')
			rospy.sleep(random_time*random.randint(1,15))
			return 'stop_sleep'
		self.rate.sleep
	## method read_actual_position
	#
   	# subscriber to actual_position_robot topic callback, it reads the actual position of the robot
   	def read_actual_position(self,position):
		self.position = position.data

## class Play_behavior
#
# This class implement the PLAY behavior of the robot pet
#
# It moves the robot to the predefined (X, Y) location within the map and moves it back to the user.
class Play_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['stop_play']
		                    )
		self.rate = rospy.Rate(1)  
		self.position = [-1,-1]
		
	## method execute
	#
	# The robot should:
	#	- go to the location where the person is
	#	- wait for a pointing gesture
	#	- go in the pointed location
	# 	- go back to the user
	# 	- wait for the next pointing gestures
	# 	- after some time return to NORMAL state

	def execute(self, userdata):
		# publish behavior "play" on the topic \behavior
		pub_behavior.publish("play") 
		# the topic is subscribed by the node poiting gesture, which send the robot to the user location
		# motion read the goal position from the poiting gesture which keeps publishing 
		# check the actual position of the robot
		rospy.Subscriber("/actual_position_robot", IntArray, self.read_actual_position)
		# wait some random time
		rospy.sleep(random_time*random.randint(30,80))
		return 'stop_play'
	## method read_actual_position
	#
   	# subscriber to actual_position_robot topic callback, it reads the actual position of the robot
   	def read_actual_position(self,position):
		self.position = position.data

def main():
	rospy.init_node("behavior_manager")

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

