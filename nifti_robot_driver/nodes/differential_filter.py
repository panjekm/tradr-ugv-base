#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_robot_driver')
import rospy
from sensor_msgs.msg import JointState
from math import radians, degrees, sqrt

class DifferentialFilter(object):
	def __init__(self):
		self.N = rospy.get_param('~samples_number', 100)
		self.s0 = rospy.get_param('~stddev_motion', radians(0.2))
		self.bias_left = self.bias_right = 0.
		self.K = 1.
		self.n = 0.
		self.last_left = self.last_right = None
		self.acc = 0.
		self.joints_pub = rospy.Publisher("/filtered_joint_states", JointState)
		rospy.Subscriber("/joint_states", JointState, self.js_cb)
	
	def js_cb(self, joints):
		left = joints.position[joints.name.index("left_track_j")]
		right = joints.position[joints.name.index("right_track_j")]
		self.n += 1
		if self.n>self.N:
			K = self.K
			n_left = (1-K)*self.last_left + K*(left-self.bias_left)
			new_pos = list(joints.position)
			new_pos[joints.name.index("left_track_j")] = n_left
			n_right = (1-K)*self.last_right + K*(right-self.bias_right)
			new_pos[joints.name.index("right_track_j")] = n_right
			joints.position = new_pos
		else:
			self.bias_left += left
			self.acc += 0.5*left*left
			self.bias_right += right
			self.acc += 0.5*right*right
			bl = self.bias_left
			br = self.bias_right
			n = self.n
			s = self.s0
			self.K = s*s/(s*s+(self.acc/n-0.5*(bl*bl+br*br)/(n*n)))
			n_left = left
			n_right = right
		self.last_left = n_left
		self.last_right = n_right
		if self.n==self.N:
			self.acc /= self.N*self.N
			self.bias_left /= self.N
			self.bias_right /= self.N
			rospy.loginfo("left bias=%f deg, right bias=%f deg, K=%f",
					degrees(self.bias_left), degrees(self.bias_right), self.K)
		self.joints_pub.publish(joints)

## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('differential_filter')
		df = DifferentialFilter()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__== '__main__':
	main()
