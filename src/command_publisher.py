#!/usr/bin/env python
import rospy, struct, numpy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt32, Float32
from geometry_msgs.msg import Twist
from scipy.signal import butter, lfilter

# Define publishers
pub_behaviorId = rospy.Publisher('/set_behaviorId', UInt32, queue_size=1)
pub_behaviorMode = rospy.Publisher('/set_behaviorMode', UInt32, queue_size=1)
pub_twist = rospy.Publisher('/set_twist', Twist , queue_size=1)

# Initialize global behavior and mode variables
behavior = 0
mode = 0

# Behaviors
BEHAVIOR_WALK      = 0
BEHAVIOR_KILL      = 1
BEHAVIOR_PUSHWALK  = 2
BEHAVIOR_MOUNT     = 3

# Modes
MODE_STAND = 0
MODE_START = 1

# Filter parameters
global LinearCmdBuffer, AngularCmdBuffer
LowpassCutoff = 4.
LowpassSampling = 25
LowpassOrder = 6
LowpassSamples = 40



# Function that generates the filter coefficients based on a cutoff frequency and a sampling rate
# Input  - 1) cutoff: Cutoff frequency
#          2) fs: Sampling frequency
#          3) order: Order of the filter
# Output - 1) b: Numerator coefficient array
#          2) a: Denominator coefficient array
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff/nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


# Function that filters the desired data based on a cutoff frequency and a sampling frequency
# Input  - 1) data: Data array
#          2) cutoff: Cutoff frequency
#          3) fs: Sampling frequency
#          4) order: Order of the filter
# Output - 1) y: Filtered version of the input data
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Main callback function
def callback(data):
	global behavior, mode
	global LinearCmdBuffer, AngularCmdBuffer

	#X
	if (data.buttons[2] == 1):
		behavior = BEHAVIOR_KILL
	#A
	elif (data.buttons[0] == 1):
		mode = MODE_STAND
	#START
	elif (data.buttons[7] == 1):
		mode = MODE_START
	#B - unused
	elif (data.buttons[1] == 1):
		a = 1
	#Y
	elif (data.buttons[3] == 1):
		behavior = BEHAVIOR_WALK
	#RB
	elif (data.buttons[5] == 1):
		behavior = BEHAVIOR_MOUNT
	#LB
	elif (data.buttons[4] == 1):
		behavior = BEHAVIOR_PUSHWALK
	
	# Define twist
	twist = Twist()

	# Define forward velocity
	LinearCmdRaw = data.axes[3]
	LinearCmdBuffer = numpy.hstack((LinearCmdBuffer[1:],numpy.array([LinearCmdRaw])))
	LinearCmdBufferFlt = butter_lowpass_filter(LinearCmdBuffer, LowpassCutoff, LowpassSampling, LowpassOrder)
	LinearCmd = LinearCmdBufferFlt[-1]
	twist.linear.x = LinearCmd

	# Define angular yaw rate
	AngularCmdRaw = data.axes[2]
	AngularCmdBuffer = numpy.hstack((AngularCmdBuffer[1:],numpy.array([AngularCmdRaw])))
	AngularCmdBufferFlt = butter_lowpass_filter(AngularCmdBuffer, LowpassCutoff, LowpassSampling, LowpassOrder)
	AngularCmd = AngularCmdBufferFlt[-1]
	twist.angular.z = AngularCmd

	# Define robot height
	HeightCmd = data.axes[1]
	twist.linear.z = HeightCmd
	
	# Log info and publish
	rospy.loginfo("%d %d %.4f %.4f %.4f", behavior, mode, LinearCmd, AngularCmd, HeightCmd)
	pub_behaviorId.publish(behavior)
	pub_behaviorMode.publish(mode)
	pub_twist.publish(twist)
	return

def init():
	global LinearCmdBuffer, AngularCmdBuffer

	# Initialize node
	rospy.init_node('command_publisher', anonymous=True)

	# Initialize linear and angular command buffers
	LinearCmdBuffer = numpy.zeros(LowpassSamples)
	AngularCmdBuffer = numpy.zeros(LowpassSamples)
	
	# Subscribe to joy commands
	rospy.Subscriber("/joy", Joy, callback)

	rospy.spin()
	
if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException: pass
