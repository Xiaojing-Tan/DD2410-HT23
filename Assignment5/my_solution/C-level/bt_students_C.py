#!/usr/bin/env python3

# Student1: Xiaojing Tan, email: xta@kth.se, ID:20010226-6798
# Student2: Ji Yang, email: ji4@kth.se, ID:20000707-3935

import re

from matplotlib.pyplot import cla
import py_trees as pt, py_trees_ros as ptr, rospy
# from behaviours_student import *
from reactive_sequence import RSequence

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from sensor_msgs.msg import JointState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
import tf2_ros
import tf2_geometry_msgs
import sys

# global variables
node_name = "Student BT"
CHECK = {"arm_tucked": False, "head_state": "unknown", "detected": False, "picked": False, "placed": False, "cube_on_table": False}

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# tuck arm
		tuck_sequence = pt.composites.Sequence(
			 name = "Tuck sequence",
			 children=[move("backward", 10), tuck_arm(), move("forward", 10)]
		)
		b0 = pt.composites.Selector(
			name = "Tuck arm?",
			children=[check_tuck(), tuck_sequence]
		)

		# lower head
		b1 = move_head("down")

		# detect cube
		b2 = detect_for_pick()

		# pick cube
		b3 = pick_cube()

		# move to second table
		b4 = move_to("second_table")

		# place cube
		b5 = place_cube()
		place_cube_sequence = pt.composites.Sequence(
			name = "Place cube sequence",
			children=[b5, move("clockwise", 30), tuck_arm(), move("counter_clockwise", 30)]
		)

		# check cube on table
		b6 = check_cube_on_table()
		check_cube = pt.composites.Selector(
			name = "Check tube?",
			children=[check_cube_on_table(), move_to("first_table")]
		)

		# reset CHECK	
		b7 = reset_CHECK()
		
		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, place_cube_sequence, check_cube, b7])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)

class check_tuck(pt.behaviour.Behaviour):
	def __init__(self):
		super(check_tuck, self).__init__("Check arm condifiton")
	def update(self):
		if CHECK["arm_tucked"]:
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.FAILURE


class reset_CHECK(pt.behaviour.Behaviour):
	def __init__(self):
		super(reset_CHECK, self).__init__("Reset CHECK")
	def update(self):
		global CHECK
		CHECK.update({"detected": False, "picked": False, "placed": False, "cube_on_table": False})
		rospy.loginfo("%s: Initial state initialized.", node_name)
		return pt.common.Status.SUCCESS


class detect_for_pick(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing detect cube...", node_name)
		# tf2 setup
		self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
		self.tf_buffer = tf2_ros.Buffer()
		self.lisener = tf2_ros.TransformListener(self.tf_buffer)
		# Instantiate publishers
		aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.aruco_pose_pub = rospy.Publisher(aruco_pose_top, PoseStamped, queue_size=10)
		# become a behaviour
		super(detect_for_pick, self).__init__("Detect for pick!")
	
	def update(self):
		global CHECK
		# real-time check topic /robotics_intro/aruco_single/pose
		try:
			# get the pose in sensor frame
			detected_pose = rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)
			# transform into robot_base_frame and publish aruco_pose_top
			trans = self.tf_buffer.lookup_transform(self.robot_base_frame, 'xtion_rgb_optical_frame', rospy.Time.now(),rospy.Duration(1))
			detected_pose = tf2_geometry_msgs.do_transform_pose(detected_pose, trans)
			rospy.loginfo("%s: Detect cube succeded!", node_name)
			self.aruco_pose_pub.publish(detected_pose)
			CHECK["detected"] = True
			return pt.common.Status.SUCCESS
		except rospy.ROSException as e:
			rospy.loginfo("%s: Detect cube failed!", node_name)
			print("Error: %s"%e)
			CHECK["detected"] = False
			return pt.common.Status.FAILURE

class check_cube_on_table(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing check place...", node_name)

		super(check_cube_on_table, self).__init__("Check place action")

	def update(self):
		global CHECK
		if CHECK["cube_on_table"]:
			rospy.loginfo("%s: Cube already place on table!", node_name)
			return pt.common.Status.SUCCESS
		else:
			try:
				rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)
				rospy.loginfo("%s: Check cube on table succeeded! End of task!", node_name)
				sys.exit(1)
				return pt.common.Status.SUCCESS
			except rospy.ROSException as e:
				rospy.loginfo("%s: Check cube on table failed!", node_name)
				# print("Error: %s"%e)
				return pt.common.Status.FAILURE

class pick_cube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing pick cube...", node_name)
		# Server
		pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.pick_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		rospy.wait_for_service(pick_srv_nm, timeout=30)
		# become a behaviour
		super(pick_cube, self).__init__("Pick cube!")

	def update(self):
		global CHECK
		# success if done
		if CHECK["picked"]:
			rospy.loginfo("%s: Cube already picked!", node_name)
			return pt.common.Status.SUCCESS
		else:
			self.pick_srv_req = self.pick_srv()
			# if succesful
			if self.pick_srv_req.success:
				CHECK["picked"] = True
				CHECK["arm_tucked"] = False
				rospy.loginfo("%s: Pick cube succeded!", node_name)
				return pt.common.Status.SUCCESS
			# if failed
			elif not self.pick_srv_req.success:
				CHECK["arm_tucked"] = False
				rospy.loginfo("%s: Pick cube failed!", node_name)
				return pt.common.Status.FAILURE
			# if still trying
			else:
				return pt.common.Status.RUNNING

class place_cube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing place cube...", node_name)
		# Server
		place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.pĺace_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
		rospy.wait_for_service(place_srv_nm, timeout=30)
		# become a behaviour
		super(place_cube, self).__init__("Place cube!")

	def update(self):
		global CHECK
		# success if done
		if CHECK["placed"]:
			rospy.loginfo("%s: Cube already placed!", node_name)
			return pt.common.Status.SUCCESS
		else:
			self.place_srv_req = self.pĺace_srv()
			# if succesful
			if self.place_srv_req.success:
				CHECK["placed"] = True
				CHECK["arm_tucked"] = False
				rospy.loginfo("%s: Place cube finished!", node_name)
				return pt.common.Status.SUCCESS
			# if failed
			elif not self.place_srv_req.success:
				CHECK["arm_tucked"] = False
				rospy.loginfo("%s: Place cube failed!", node_name)
				return pt.common.Status.FAILURE
			# if still trying
			else:
				return pt.common.Status.RUNNING

class move(pt.behaviour.Behaviour):
	def __init__(self, direction, steps):
		self.rate = rospy.Rate(10)
		self.move_msg = Twist()
		self.direction = direction
		self.steps = steps
		if self.direction == "forward":
			self.move_msg.linear.x = 0.2
		elif self.direction == "backward":
			self.move_msg.linear.x = -0.5
		elif self.direction == "clockwise":
			self.move_msg.angular.z = -1
		elif self.direction == "counter_clockwise":
			self.move_msg.angular.z = 1
		# Instantiate publishers
		cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
		# become a behaviour
		super(move, self).__init__("Move!")

	def update(self):
		rospy.loginfo("%s: Moving %s...", node_name, self.direction)
		cnt = 0
		while not rospy.is_shutdown() and cnt < self.steps:
			self.cmd_vel_pub.publish(self.move_msg)
			self.rate.sleep()
			# print(cnt)
			cnt += 1
		# stop after finish
		self.move_msg = Twist()
		self.cmd_vel_pub.publish(self.move_msg)
		rospy.sleep(1)
		rospy.loginfo("%s: Done", node_name)
		return pt.common.Status.SUCCESS

class move_to(pt.behaviour.Behaviour):
	def __init__(self, goal):
		rospy.loginfo("%s: Initializing move to...", node_name)
		self.goal = goal
		self.reach_goal = False
		self.direction = 1
		if goal == "second_table":
			self.direction = 1
		elif goal == "first_table":
			self.direction = -1
		self.rate = rospy.Rate(10)
		self.move_msg = Twist()
		# Instantiate publishers
		cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
		# become a behaviour
		super(move_to, self).__init__("Move to goal!")

	def update(self):
		rospy.loginfo("%s: Moving towards the %s...", node_name, self.goal)
		# turn around
		self.move_msg.angular.z = -1 * self.direction
		cnt = 0
		while not rospy.is_shutdown() and cnt < 33:
			self.cmd_vel_pub.publish(self.move_msg)
			self.rate.sleep()
			cnt += 1
		# pause
		self.move_msg.angular.z = 0
		self.cmd_vel_pub.publish(self.move_msg)
		rospy.sleep(1)
		# go straight forward
		self.move_msg.linear.x = 0.5
		cnt = 0
		while not rospy.is_shutdown() and cnt < 20:
			self.cmd_vel_pub.publish(self.move_msg)
			self.rate.sleep()
			cnt += 1
		# stop before the second table
		self.move_msg.linear.x = 0
		self.move_msg.angular.z = 0
		self.cmd_vel_pub.publish(self.move_msg)
		rospy.loginfo("%s: Reached the %s", node_name, self.goal)
		return pt.common.Status.SUCCESS

class tuck_arm(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising tuck arm behaviour.")
		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True
		# become a behaviour
		super(tuck_arm, self).__init__("Tuck arm!")

	def update(self):
		global CHECK
		# already tucked the arm
		if CHECK["arm_tucked"]:
			rospy.loginfo("%s: Arm already tucked", node_name)
			return pt.common.Status.SUCCESS
		else:
			# send the goal
			self.play_motion_ac.send_goal(self.goal)
			# if I was succesful! :)))))))))
			while not self.play_motion_ac.get_result():
				pass
			# then I'm finished!
			CHECK["arm_tucked"] = True
			# rospy.sleep(10) # wait for arm to tuck in gazebo
			rospy.loginfo("%s: Tuck arm succeeded", node_name)
			return pt.common.Status.SUCCESS
			# if failed
			# elif not self.play_motion_ac.get_result():
			# 	rospy.loginfo("%s: Tuck arm failed", node_name)
			# 	return pt.common.Status.FAILURE
			# # if I'm still trying :|
			# else:
			# 	return pt.common.Status.RUNNING

class move_head(pt.behaviour.Behaviour):
	def __init__(self, direction):
		rospy.loginfo("Initialising move head behaviour.")
		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)
		# head movement direction; "down" or "up"
		self.direction = direction
		# become a behaviour
		super(move_head, self).__init__("Move head!")

	def update(self):
		global CHECK
		if CHECK["head_state"] == self.direction:
			rospy.loginfo("%s: Head already %s", node_name, CHECK["head_state"])
			return pt.common.Status.SUCCESS
		else:
			self.move_head_req = self.move_head_srv(self.direction)
			# if succesful
			if self.move_head_req.success:
				rospy.sleep(3) # wait for head move in gazebo
				CHECK["head_state"] = self.direction
				print("Head", self.direction, "succeeded")
				return pt.common.Status.SUCCESS
			# if failed
			elif not self.move_head_req.success:
				print("Head",self.direction,"failed!!!")
				return pt.common.Status.FAILURE
			# if still trying
			else:
				return pt.common.Status.RUNNING

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
