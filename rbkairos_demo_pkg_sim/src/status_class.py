#!/usr/bin/env python

import rospy
from rbkairos_demo_pkg_sim.msg import Status

class status_publisher():
	publisher = None

	cur_status = {'arm':'stopped', 'base':'stopped', 'gripper':'stopped'}
	# Possible status
	# Base: stopped, moving; (0, 1)
	# Arm: stopped, moving; (0, 1)
	# Gripper: stopped, grasping, open; (0, 1, 2)

	def __init__(self):
		self.publisher = rospy.Publisher('univr/robot_status', Status, queue_size=10)
	
	# integers from 0 to 1, representing the status listed above
	def status_update_base(self, base):
		if base:
			self.cur_status['base'] = 'moving'
		else:
			self.cur_status['base'] = 'stopped'
		self.publish_status()

	# integers from 0 to 1, representing the status listed above
	def status_update_arm(self, arm):
		if arm:
			self.cur_status['arm'] = 'moving'
		else:
			self.cur_status['arm'] = 'stopped'
		self.publish_status()

	# integers from 0 to 2, representing the status listed above
	def status_update_gripper(self, gripper):
		if gripper == 0:
			self.cur_status['gripper'] = 'stopped'
		elif gripper == 1:
			self.cur_status['gripper'] = 'grasping'
		else:
			self.cur_status['gripper'] = 'open'
		self.publish_status()
	
	def publish_status(self):
		msg = Status()
		msg.base_status = self.cur_status['base']
		msg.arm_status = self.cur_status['arm']
		msg.gripper_status = self.cur_status['gripper']
		self.publisher.publish(msg)
		rospy.sleep(0.1)
	
