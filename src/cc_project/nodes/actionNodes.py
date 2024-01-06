#!/usr/bin/env python
#coding=utf-8

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
import math
import cv2
import numpy as np


def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
	"""

	d1 = 12.0; # cm (height of 2nd joint)
	a1 = 0; # (distance along "y-axis" to 2nd joint)
	a2 = 17.2; # (distance between 2nd and 3rd joints)
	d4 = 20.0; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Insert code here!!!
	# inspiration fra CrustInvKin.m

	x, y, z = xyz

	q1 = math.atan2(y, x)

	r2 = (x - a1*math.cos(q1))**2 + (y - a1*math.sin(q1))**2
	
	s = (z - d1)

	D = (r2 + s**2 - a2**2 - d4**2) / (2*a2*d4)

	q3 = math.atan2(-(math.sqrt(1-D**2)), D)
	q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2 + d4*math.cos(q3))

	q4=0

	return q1,q2,q3,q4


class camNode:
	def __init__(self,video_address):
		rospy.init_node('cameraNode')
		self.rate = rospy.Rate(1)	# ikke sikker på om det er nødvendigt
									# når den kun skal publish én gang
		self.coordinate_publisher = rospy.Publisher('Coords', Point, queue_size=10)

		self.capture = cv2.VideoCapture(video_address)

	def findBall(self):
		lowerOrange = np.array([5, 100, 100]) #nedre afgrænsning for orange i HSV
		upperOrange = np.array([15, 255, 255]) #øvre afgrænsning for orange i HSV
		while not rospy.is_shutdown():

			ret, frame = self.capture.read()

			if not ret:
				break

			frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(frameHSV, lowerOrange, upperOrange)
			# Find cirkler
			contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			for contour in contours:
				area = cv2.contourArea(contour)
				if area > 10:
					M = cv2.moments(contour)
					if M["m00"] != 0:
						circlex = int(M["m10"] / M["m00"])
						circley = int(M["m01"] / M["m00"])

						coordinates_msg = Point()

						coordinates_msg.x = circlex - 154 #origo i midten af robotten
						coordinates_msg.y = circley - 187 #origo i midten af robotten
						coordinates_msg.z = 0 # højden af bolden bliver hardcodet

						self.coordinate_publisher.publish(coordinates_msg)




class robotNode:
	N_JOINTS = 4

	def __init__(self,server_name):
		rospy.init_node('robotNode')
		self.coordinateSubscriber = rospy.Subscriber("Coords", Point, self.goToObject())
		
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

		self.joint_positions = []
		self.names = ["joint1",
				"joint2",
				"joint3",
				"joint4"]
	
	def goToObject(self, coord_msg):
		z_ballCenter = 5

		dur = rospy.Duration(1)

		x = (coord_msg.x / 4)
		y = (coord_msg.y / 4) # dividerer med 4 da det er forholdet mellem pixel/cm

		xyzBall = [x, y, z_ballCenter]

		jtp = JointTrajectoryPoint(positions=invkin(xyzBall),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
		dur += rospy.Duration(2)
		self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def send_command(self):
		self.client.wait_for_server()
		print self.goal
		self.client.send_goal(self.goal)

		self.client.wait_for_result()
		print self.client.get_result()

	def goToEndPos(self):
		joint_positions = [0.0, 1.4, -1.5, 0]

		jtp = JointTrajectoryPoint(positions=joint_positions,velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
		dur += rospy.Duration(5)
		self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def goToStartPos(self):
		joint_positions = [-1.5, 0.0, -0.2, 0]

		jtp = JointTrajectoryPoint(positions=joint_positions,velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
		dur += rospy.Duration(5)
		self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )






		

if __name__ == "__main__":
	cam_node= camNode('http://192.168.0.20/video.cgi')
	arm_node= robotNode("/arm_controller/follow_joint_tragectory")

	arm_node.goToStartPos()
	arm_node.send_command()
	
	if cam_node.findBall():
		arm_node.send_command()

	arm_node.goToEndPos()
	arm_node.send_command()



