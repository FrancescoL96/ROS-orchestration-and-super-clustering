#!/usr/bin/env python

import moveit_commander
import rospy
import actionlib
import tf

from wsg_50_common.srv import Move
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import status_class

class rbkairos_demo_pkg_sim():

	arm_group = None
	move_base_client = None
	status = None
	
	def __init__(self):
		self.status = status_class.status_publisher()

		self.arm_group = moveit_commander.MoveGroupCommander('arm')
		self.arm_group.set_planning_time(2.)
		self.move_base_client = actionlib.SimpleActionClient('/robot/move_base', MoveBaseAction)

	def move_base_target(self, x, y, theta, wait = True):
		#print("Move base to target")
		self.move_base_client.wait_for_server()
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "robot_map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.x = quaternion[0]
		goal.target_pose.pose.orientation.y = quaternion[1]
		goal.target_pose.pose.orientation.z = quaternion[2]
		goal.target_pose.pose.orientation.w = quaternion[3]
		self.status.status_update_base(1)
		self.move_base_client.send_goal(goal)
		wait_base = self.move_base_client.wait_for_result()
		self.status.status_update_base(0)
		if (wait):
			rospy.sleep(1)

	def arm_move_joints(self, target, wait_global = True):
		#print("Move arm to joints")
		reordered_target = [] # The real robot joints are ordered differently from the simulated one (the arm ones are inverted)
		reordered_target.append(target[2])
		reordered_target.append(target[1])
		reordered_target.append(target[0])
		reordered_target.append(target[3])
		reordered_target.append(target[4])
		reordered_target.append(target[5])
		self.status.status_update_arm(1)
		self.arm_group.set_joint_value_target(reordered_target)
		plan = self.arm_group.plan()
		response = self.arm_group.execute(plan, wait = True)
		self.status.status_update_arm(0)
		if (wait_global):
			rospy.sleep(1)

	def arm_move_to_target(self, target, wait_global = True):
		#print("Move arm to target")
		position = self.arm_group.get_current_pose()
		position.pose.position.x = target[0] #target.posx
		position.pose.position.y = target[1] #target.posy
		position.pose.position.z = target[2] #target.posz
		position.pose.orientation.x = target[3] #target.orx
		position.pose.orientation.y = target[4] #target.ory
		position.pose.orientation.z = target[5] #target.orz
		position.pose.orientation.w = target[6] #target.orw
		self.status.status_update_arm(1)
		self.arm_group.set_pose_target(position)
		plan = self.arm_group.plan()
		response = self.arm_group.execute(plan, wait = True)
		self.status.status_update_arm(0)
		if (wait_global):
			rospy.sleep(1)
	
	def gripper_open(self, wait = True):
		#print("Gripper open")
		rospy.wait_for_service('/robot/wsg_50/grasp')
		try:
			grasp = rospy.ServiceProxy('/robot/wsg_50/grasp', Move)
			resp = grasp(110.0, 0.0)
			self.status.status_update_gripper(2)
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)
		if (wait):
			rospy.sleep(1)

	def gripper_grasp(self, wait = True):
		#print("Gripper close")
		rospy.wait_for_service('/robot/wsg_50/grasp')
		try:
			grasp = rospy.ServiceProxy('/robot/wsg_50/grasp', Move)
			resp = grasp(0.0, 0.0)
			self.status.status_update_gripper(1)
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)
		if (wait):
			rospy.sleep(1)


def execute(robot):
	#--- start actions ---------------------------------------------------------
	robot.arm_move_joints([-1.55, -0.69, 0.00, -0.93, 1.61, -1.54])

	# Random movements to improve localization
	#robot.move_base_target(-4.089, 2.971, 0.0)
	#robot.move_base_target(4.83, 1.358, 0.0)
	#robot.move_base_target(0.0, 0.0, 0.0)

	robot.move_base_target(0.0249, -2.2001, 0.045)
	#robot.align_base
	robot.arm_move_joints([-0.72, -1.73, 0.18, -2.27, 1.57, 0.16])
	robot.gripper_open()

	#--- pick_two_parts_from_pallet ---------------------------------------------------------
	#robot.arm_move_to_target([0.655409501597, -0.0234193063112, 1.05085930488, 0.716218026628, -0.697754076151, -0.0130411018327, 0.000957711109038], False)
	#robot.arm_move_to_target([0.71191979012, -0.0174321055852, 1.05470133419, 0.714413299551, -0.699646155455, -0.00585272704106, 0.0086394541325])
	#robot.gripper_grasp()
	#robot.arm_move_to_target([0.655409501597, -0.0234193063112, 1.05085930488, 0.716218026628, -0.697754076151, -0.0130411018327, 0.000957711109038], False)
	#robot.arm_move_joints([-1.10, -1.18, -1.55, -0.93, 1.61, -1.54], False)
	#robot.arm_move_joints([-2.22, -1.37, -3.10, -1.18, 1.51, -1.53], False)
	#robot.arm_move_to_target([-0.179656502181, 0.0374707806973, 0.693368016229, 0.00405456427985, -0.999692358284, -0.0215292554411, 0.011629293024], False)
	#robot.arm_move_to_target([-0.175503332046, -0.0340447957561, 0.687599084532, -0.0202630711247, -0.999519977181, -0.0206915405411, 0.0110037863719])
	#robot.gripper_open()
	#robot.arm_move_to_target([-0.179656502181, 0.0374707806973, 0.693368016229, 0.00405456427985, -0.999692358284, -0.0215292554411, 0.011629293024], False)
	#robot.arm_move_joints([-1.10, -1.18, -1.55, -0.93, 1.61, -1.54], False)
	#robot.arm_move_joints([-0.72, -1.73, 0.18, -2.27, 1.57, 0.16], False)
	#robot.arm_move_to_target([0.655409501597, -0.0234193063112, 1.05085930488, 0.716218026628, -0.697754076151, -0.0130411018327, 0.000957711109038], False)
	robot.arm_move_to_target([0.71191979012, -0.0174321055852, 1.05470133419, 0.714413299551, -0.699646155455, -0.00585272704106, 0.0086394541325])
	robot.gripper_grasp()
	#robot.arm_move_to_target([0.655409501597, -0.0234193063112, 1.05085930488, 0.716218026628, -0.697754076151, -0.0130411018327, 0.000957711109038], False)
	robot.arm_move_joints([-0.72, -1.73, 0.18, -2.27, 1.57, 0.16], False)
	 
	#--- place_two_parts_to_warehouse ---------------------------------------------------------
	robot.arm_move_joints([-1.55, -0.69, 0.00, -0.93, 1.61, -1.54])
	robot.move_base_target(0.0249, -2.2001, 0.045) #unalign
	robot.move_base_target(0.1826, -1.5124, 1.6946)
	#robot.align_base
	robot.arm_move_joints([-1.17, -1.38, -0.00, -0.60, 1.56, -1.59], False)
	robot.arm_move_joints([-0.69, -2.01, 0.00, -1.02, 1.56, -1.59])
	robot.gripper_open()
	#robot.arm_move_joints([-1.17, -1.38, -0.00, -0.60, 1.56, -1.59], False)
	#robot.arm_move_joints([-1.10, -1.18, -1.55, -0.93, 1.61, -1.54], False)
	#robot.arm_move_joints([-2.22, -1.37, -3.10, -1.18, 1.51, -1.53], False)
	#robot.arm_move_to_target([-0.179656502181, 0.0374707806973, 0.693368016229, 0.00405456427985, -0.999692358284, -0.0215292554411, 0.011629293024], False)
	#robot.arm_move_to_target([-0.175503332046, -0.0340447957561, 0.687599084532, -0.0202630711247, -0.999519977181, -0.0206915405411, 0.0110037863719])
	#robot.gripper_grasp()
	#robot.arm_move_to_target([-0.179656502181, 0.0374707806973, 0.693368016229, 0.00405456427985, -0.999692358284, -0.0215292554411, 0.011629293024], False)
	#robot.arm_move_joints([-1.10, -1.18, -1.55, -0.93, 1.61, -1.54], False)
	#robot.arm_move_joints([-1.17, -1.38, -0.00, -0.60, 1.56, -1.59], False)
	#robot.arm_move_joints([-0.69, -2.01, 0.00, -1.02, 1.56, -1.59])
	#robot.gripper_open() # Questo era move_gripper_duplo??
	robot.arm_move_joints([-1.17, -1.38, -0.00, -0.60, 1.56, -1.59], False)
	robot.arm_move_joints([-1.55, -0.69, 0.00, -0.93, 1.61, -1.54])


	#--- post_place_warehouse---------------
	robot.move_base_target(0.1826, -1.5124, 1.6946) # unalign
	robot.move_base_target(0.0249, -2.2001, 0.045)
	#align_base
	robot.arm_move_joints([-0.72, -1.73, 0.18, -2.27, 1.57, 0.16])
	robot.gripper_open()

if __name__ == '__main__':
	rospy.init_node('rbkairos_demo_pkg_sim')
	try:
		demo = rbkairos_demo_pkg_sim()
		execute(demo)
	except rospy.ROSInterruptException: pass
