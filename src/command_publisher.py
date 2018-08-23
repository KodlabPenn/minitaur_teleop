#!/usr/bin/env python
import rospy, struct
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt32, Float32
from geometry_msgs.msg import Twist

# Define publishers
pub_behaviorId = rospy.Publisher('/set_behaviorId', UInt32, queue_size=1)
pub_behaviorMode = rospy.Publisher('/set_behaviorMode', UInt32, queue_size=1)
pub_twist = rospy.Publisher('/set_twist', Twist , queue_size=1)

# Initialize global behavior and mode variables
behavior = 0
mode = 0

# Available commands
BEHAVIOR_BOUND = 0
BEHAVIOR_WALK = 1
MODE_STAND = 0
MODE_START = 1

def callback(data):
	global behavior, mode

	#X - unused
	if (data.buttons[2] == 1):
		a = 1
	#A
	elif (data.buttons[0] == 1):
		mode = MODE_STAND
	#START
	elif (data.buttons[7] == 1):
		mode = MODE_START
	#B
	elif (data.buttons[1] == 1):
		behavior = BEHAVIOR_BOUND
	#Y
	elif (data.buttons[3] == 1):
		behavior = BEHAVIOR_WALK
	#RB - unused
	elif (data.buttons[5] == 1):
		a = 1
	#LB
	elif (data.buttons[4] == 1):
		a = 1
	
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
	rospy.loginfo("%d %d %.4f %.4f %.4f", behavior, mode, lin_x, angular_z, height)
	pub_behaviorId.publish(behavior)
	pub_behaviorMode.publish(mode)
	pub_twist.publish(twist)
	return

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
