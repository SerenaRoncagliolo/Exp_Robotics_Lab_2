#! /usr/bin/env python

## @package go_to_point_robot
#
# implement an action server to move the robot on the map

# import ros stuff
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import math
import actionlib
import actionlib.msg
import assignment2.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0 # to stop it
# parameters
yaw_precision_ = math.pi / 9 # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = 3.0
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6
#z_back = 0.25

# publisher
pub = None

# action_server
act_s = None

## publisher
# joint_pub is used  to move the head of the robot 
joint_pub = rospy.Publisher("joint_head_controller/command", Float64, queue_size = 1)

## callbacks
#
## function clbk_odom
#
# callback function dedicated to the odometry of the robot
def clbk_odom(msg):
	global position_
	global pose_
	global yaw_
    
	# position
	position_ = msg.pose.pose.position
	pose_ = msg.pose.pose
    
	# yaw
	quaternion = (
	        msg.pose.pose.orientation.x,
        	msg.pose.pose.orientation.y,
        	msg.pose.pose.orientation.z,
        	msg.pose.pose.orientation.w)
    	euler = transformations.euler_from_quaternion(quaternion)
    	yaw_ = euler[2]


## function change_state
#
# function which changes the state while the robot moves until it reaches the goal
def change_state(state):
	global state_
	state_ = state
	print('ACTION SERVER ROBOT: state changed to [%s]' % state_)

## function normalize_angle
#
# function to normalize the robot angle
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
        	angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    	return angle

## function yaw
#
# fixes the yaw of the robot
def fix_yaw(des_pos):
	global yaw_, pub, yaw_precision_2_, state_
   	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = normalize_angle(desired_yaw - yaw_)
    	rospy.loginfo(err_yaw)

	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
        	twist_msg.angular.z = kp_a*err_yaw 
        if twist_msg.angular.z > ub_a:
		twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a: 
		twist_msg.angular.z = lb_a
    
	pub.publish(twist_msg)
    
    	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:		
		print('ACTION SERVER ROBOT: Yaw error is [%s]' % err_yaw)
		# go to the next state 
        	change_state(1)

## function go_straight_ahead
#
# function which moves the robot straight to the goal
def go_straight_ahead(des_pos):	
	global yaw_, pub, yaw_precision_, state_

    	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    	err_yaw = desired_yaw - yaw_
    	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        	pow(des_pos.x - position_.x, 2))
    	err_yaw = normalize_angle(desired_yaw - yaw_)
    	rospy.loginfo(err_yaw)

	if err_pos > dist_precision_:
        	twist_msg = Twist()
        	twist_msg.linear.x = 0.3
        	if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d
        
        	twist_msg.angular.z = kp_a*err_yaw
        	pub.publish(twist_msg)
    	else:
        	print('ACTION SERVER ROBOT: Position error: [%s]' % err_pos)
        	change_state(2)
    
	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
        	print('Yaw error: [%s]' % err_yaw)
        	change_state(0)

## function done
#
# it set the robot velocities to zero
def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0
	# twist_msg.linear.Y = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)
    
## function planning
#
# function which computes what the robot should do
def planning(goal):
	
	# initialization 
	global state_, desired_position_
	global act_s
	
	# set position according to desired position
	desired_position_.x = goal.target_pose.pose.position.x
	desired_position_.y = goal.target_pose.pose.position.y

	# initialize the other fields of the action server 
	state_ = 0
	rate = rospy.Rate(20)
	success = True

	# initialize feedback and result for action server
	feedback = assignment2.msg.PlanningFeedback()
	result = assignment2.msg.PlanningResult()
	
	# while loop used to reach the goal
	while not rospy.is_shutdown():
		# check if cancel command was sent
		if act_s.is_preempt_requested():
			rospy.loginfo('ACTION SERVER ROBOT: Goal was preempted')
			act_s.set_preempted()
			success = False
			break
		# if we are at initial state, we adjust the yaw of the robot
		elif state_ == 0:
			feedback.stat = "Fixing the yaw"
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			# call fix_yaw function passing it the desired_position (= goal)
			fix_yaw(desired_position_)
			# go straight until the desired position is reached
		elif state_ == 1:
			feedback.stat = "Angle aligned"
			# keep robot head straight
			time.sleep(1)
			joint_pub.publish(0)
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			# call go_straight_ahead function to reach the desired position 
			go_straight_ahead(desired_position_)
			# end state
		elif state_ == 2:
			feedback.stat = "Target reached!"
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			done()
			break
		else:
		    rospy.logerr('ACTION SERVER ROBOT: Unknown state!')
		
		rate.sleep()
	if success:
		rospy.loginfo('ACTION SERVER ROBOT: Robot goal: Succeeded!')
		act_s.set_succeeded(result)

## function main
#
def main():
	global pub, active_, act_s
	rospy.init_node('go_to_point')
	# publish velocity in Gazebo
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	# subscribe Gazebo odometry
	sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)
	
	# action Server
	act_s = actionlib.SimpleActionServer(
        	'robot_reaching_goal', assignment2.msg.PlanningAction, planning, auto_start=False)
    	act_s.start()

    	rate = rospy.Rate(20)
    
    	while not rospy.is_shutdown():
        	rate.sleep()


if __name__ == '__main__':
	main()
