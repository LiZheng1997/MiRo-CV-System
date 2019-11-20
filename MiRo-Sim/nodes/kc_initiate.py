# -*- coding: utf-8 -*-
#!/usr/bin/python
import os
import math
import numpy as np
import time
import miro2 as miro
from threading import Thread

# ROS d
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Kc_init:

	def __init__(self,name):

		#robot name
		self.topic_root = name
		# state
		# create kc object with default (calibration) configuration
		# of joints (and zeroed pose of FOOT in WORLD)
		self.kc = miro.utils.kc_interf.kc_miro()
		# self.cam = miro.utils.camera_model.CameraModel()
		# self.pose = np.array([0.0, 0.0, 0.0])
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"] #lift默认为34度角,改成40度
		self.cos_joints = Float32MultiArray()
		# [droop, wag, eyel, eyer, earl, earr]
		self.cosmetic_joints_pub = rospy.Publisher(self.topic_root + "control/cosmetic_joints", Float32MultiArray, queue_size=1)
		self.kinematic_joints_pub = rospy.Publisher(self.topic_root + "control/kinematic_joints", JointState, queue_size=1)


	def init_kc(self):
		# sleep (10Hz)
		# time.sleep(1)
		print("intitate the start kinematic status")
		self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
		#here i set a initiate status for the robot, to get a view lower, focusing more
		#on the obejcts on the floor, because our aims are MiRos and balls, sometimes 
		# pedestrians, so the angles have been tested, do not change this.
		self.kin_joints.position = [0.0, math.radians(40.0), 0.0,  math.radians(0.0)]
		# publish
		time.sleep(1)
		self.kinematic_joints_pub.publish(self.kin_joints)
		# publish
		time.sleep(1)
		self.cosmetic_joints_pub.publish(self.cos_joints)
		self.kc.setConfig(self.kin_joints.position)

	#this function can set each kinematic joint of the neck, we have 4 joints.
	def set_neck(self, joint_index, degrees):
		#self.pause()
		if joint_index >= 1 and joint_index < 4:
			if joint_index == miro.constants.JOINT_LIFT:
				degrees = np.clip(degrees, 5, 60)
			elif joint_index == miro.constants.JOINT_PITCH:
				degrees = np.clip(degrees, -22, 8)
			elif joint_index == miro.constants.JOINT_YAW:
				degrees = np.clip(degrees, -60, 60)

			joint_radians = math.radians(degrees)
			self.kin_joints.position[joint_index] = joint_radians
	#This function can set each cosmetic joint in MIRO, we have 6 joints.
	#left ear, right ear, left eye , right eye, droop, wag.
	def set_joint(self, joint_index, pos):
		#self.pause()
		if 0 <= joint_index < 6:
			pos = np.clip(pos, 0.0, 1.0)
			self.cos_joints.data[joint_index] = pos

