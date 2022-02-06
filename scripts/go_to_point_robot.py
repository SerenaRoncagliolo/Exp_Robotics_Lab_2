#! /usr/bin/env python

## @package go_to_point_robot
#
# implement an action server to move the robot on the map

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg
import exp_assignment2.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0 # stop
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = 3.0
kp_d = 0.5
ub_a = 0.6
lb_a = -0.5
ub_d = 2.0
z_back = 0.25

# publisher
pub = None
pub_z = None

# action_server
act_s = None

# callbacks

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
	
	# yaw computation
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
    print ('ACTION SERVER ROBOT: State changed to [%s]' % state_)

## function normalize_angle
#
# function to normalize the robot angle
def normalize_angle(state):
    if(math.fabs(angle) > math.pi):
	# normalization
	angleNormal = angle - (2*math.pi * angle) / (math.fabs(angle))
	return angleNormal

## function yaw
#
# function to set the yaw angle of the robot
def yaw(goalPosition):
	global yaw_, pub, yaw_precision_2_, state_
	goalPos = math.atan2(goalPosition.y - position_.y, goalPosition.x - position_.x)
    	error_yaw = normalize_angle(goalPos - yaw_)

	twist_msg = Twist()
	
    	if math.fabs(error_yaw) > yaw_precision_2_:

        	twist_msg.angular.z = kp_a*error_yaw 

		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a: 
			twist_msg.angular.z = lb_a
	    
	pub.publish(twist_msg)

	# state change 
	if math.fabs(error_yaw) <= yaw_precision_2_:
		print('ACTION SERVER ROBOT: Yaw error is : [%s]' % error_yaw)
		change_state(1) # change state function

## function go_straight_ahead
#
# function which moves the robot straight to the goal
def go_straight_ahead(goalPos):
	global yaw_, pub, yaw_precision_, state_
    
	goalYaw = math.atan2(goalPos.y - position_.y, goalPos.x - position_.x)
	err_yaw = goalYaw - yaw_
	err_pos = math.sqrt(pow(goalPos.y - position_.y, 2) + pow(goalPos.x - position_.x, 2))
	err_yaw = normalize_angle(goalYaw - yaw_)
	
	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.5
		if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d

		twist_msg.angular.z = kp_a*err_yaw
        	pub.publish(twist_msg)
	else:
		print('ACTION SERVER ROBOT: Position error equal to: [%s]' % err_pos)
		change_state(2)

	# state change conditions
    	if math.fabs(err_yaw) > yaw_precision_:
		print('ACTION SERVER ROBOT: Yaw error equal to: [%s]' % err_yaw)
		change_state(0)

## function done
#
# it set the robot velocities to zero
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    pub.publish(twist_msg)

## function planning
#
# function which computes what the robot should do
def planning(goal):

    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y


    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = exp_assignment2.msg.PlanningFeedback()
    result = exp_assignment2.msg.PlanningResult()

    while not rospy.is_shutdown():
    	if act_s.is_preempt_requested():
        	rospy.loginfo('ACTION SERVER ROBOT: Goal was preempted')
    	        act_s.set_preempted()
    	        success = False
    	        break
        elif state_ == 0:
    	        feedback.stat = "ACTION SERVER ROBOT: Fix yaw angle"
    	        feedback.position = pose_
    	        act_s.publish_feedback(feedback)
    	        fix_yaw(desired_position_)
        elif state_ == 1:
    	        feedback.stat = "Target position reached"
    	        feedback.position = pose_
    	        act_s.publish_feedback(feedback)
		go_straight_ahead(desired_position_)
    	        done()
    	        break
	elif state_ == 2:
		feedback.stat = "Target position reached"!"
    	        feedback.position = pose_
    	        act_s.publish_feedback(feedback)
    	        done()
    	        break
        else:
    	        rospy.logerr('ACTION SERVER ROBOT: Unknown state!')
        rate.sleep()
    if success:
        rospy.loginfo('ACTION SERVER ROBOT: Goal reached successfully')
        act_s.set_succeeded(result)

## function main
#
def main():
    # variables
    global pub, active_, act_s, pubz

    rospy.init_node('go_to_point_robot')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pubz = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)
    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/reaching_goal', exp_assignment2.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
