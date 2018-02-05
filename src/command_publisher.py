#!/usr/bin/env python
import rospy, struct
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, Float32
from geometry_msgs.msg import Twist

# Define publishers
pub1 = rospy.Publisher('/set_cmd', UInt8, queue_size=100)
pub2 = rospy.Publisher('/set_twist', Twist , queue_size = 100)

# Initialize state
state = 0

# Available commands
CMD_KILL = 0
CMD_STAND = 1
CMD_START = 2
CMD_SET_POSITION = 3
CMD_SET_OPEN_LOOP = 4
CMD_SET_GAIT_BOUND = 5
CMD_SET_GAIT_WALK = 6
CMD_SET_GAIT_PUSH = 7
CMD_SET_GAIT_JUMPON = 8
CMD_SIGNAL0 = 10
CMD_SIGNAL1 = 11
CMD_SIGNAL2 = 12
CMD_SIGNAL3 = 13

def callback(data):
	global state 
	r = rospy.Rate(1000)
	
	#X
	if (data.buttons[2] == 1):
		state = CMD_KILL
	#A
	elif (data.buttons[0] == 1):
		state = CMD_STAND
	#START
	elif (data.buttons[7] == 1):
		state = CMD_START
	#B
	elif (data.buttons[1] == 1):
		state = CMD_SET_GAIT_BOUND
	#Y
	elif (data.buttons[3] == 1):
		state = CMD_SET_GAIT_WALK
	#RB
	elif (data.buttons[5] == 1):
		state = CMD_SET_GAIT_JUMPON
	#LB
	elif (data.buttons[4] == 1):
		state = CMD_SET_GAIT_PUSH
	
	# Define twist
	twist = Twist()

	# Define forward velocity
	lin_x = data.axes[4] * 0.5 
	twist.linear.x = lin_x

	# Define angular yaw rate
	angular_z = -data.axes[3] * 0.15
	twist.angular.z = angular_z

	# Define robot height
	height_ref = data.axes[1] * 0.5
	height = (height_ref+1.)/2.
	twist.linear.z = height
	
	# Log info and publish
	rospy.loginfo("%d %.4f %.4f %.4f", state, lin_x, angular_z, height)
	pub1.publish(state)
	pub2.publish(twist)

def init():
	
	# Initialize node
	rospy.init_node('command_publisher', anonymous=True)
	
	# Subscribe to joy commands
	rospy.Subscriber("/joy", Joy, callback)

	rospy.spin()
	
if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException: pass
