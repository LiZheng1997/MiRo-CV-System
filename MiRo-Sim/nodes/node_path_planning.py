# python
# -*- coding: utf-8 -*-
import os
import math
import numpy as np
import time
import miro2 as miro
from threading import Thread

# ROS
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError
from node_detector import *
from transform import *
from data_resolver import *

import node_change_kc

class PathPlanner:

	def __init__(self,name):
		# topic root
		self.topic_root = name 

		self.cam = miro.lib.camera_model.CameraModel()
		# self.pose = np.array([0.0, 0.0, 0.0])
		#self.cam.set_frame_size(320, 180)
		# self.detector = Detector(name)
		self.end_point = []
		self.dfovea_WORLD = []
		self.kc_updater = node_change_kc.ChangeKc(name)
		self.transformer = Transformer(name)
		self.data_resolver = DataResolver()
		self.kinematic_joints = np.array([])
		#get the loc of sonar on head.
		self.fovea_HEAD = miro.lib.get("LOC_SONAR_FOVEA_HEAD")
		# self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

		#Configure ROS interface
		#Publishers
		self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
		self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)
		
		#Subscribers
		self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.callback_sensors)
		self.kin_sub = rospy.Subscriber(self.topic_root + "sensors/kinematic_joints", JointState,self.callback_kin)
		#Create objects to hold published data
		self.velocity = TwistStamped()

	def path_planning_push(self,ow):

		#when doing the path planning, we have to assure the location of the head of the robot
		#so, we have to initiate a location of the nose tip in the world coordination.
		# create objects in HEAD
		#we need to use the current kc not the default one
		self.kc_updater.change_kc(self.kinematic_joints)
		self.end_point = ow
		fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, self.end_point)
		self.push_to_point(fovea_x_HEAD)


	def path_planning(self,cam_index,center_pos,old_t_pos,beta,pose,now,old_time,angular_vel):

		time = 0.0
		#calculate the angle of target, the angle is theta_t, it is between the 
		#x-axis direction and the velovlity direction. Using the old centerpos 
		#and the current centerpos in pixels.
		t_pos = self.transformer.pixel_to_world(cam_index, center_pos,pose )
		dif_x = abs(t_pos[0] - old_t_pos[0])
		dif_y = abs(t_pos[1] - old_t_pos[1])
		#这里求解theta_t，用x轴的坐标除y轴，求解tan值，是因为机器人中世界坐标系都是右手定则，
		#所以，我们需要转换x,y，。
		t_move_distance = self.data_resolver.distance_resolver(old_t_pos,t_pos)
		print("t_move_distance--------------------------------->",t_move_distance)
		theta_t = self.data_resolver.angle_resolver(dif_x, dif_y, "tan")
		
		#dirca is the angle of target, it is between the 
		#sight line direction and the velovlity direction.
		lambda_angle = beta
		dirca = theta_t - lambda_angle

		cos_d_l = np.cos(theta_t)
		sin_d_l = np.sin(theta_t)
		cos_l = np.cos(lambda_angle)
		sin_l = np.sin(lambda_angle)

		vel_t = t_move_distance / (now-old_time)
		if vel_t > 0.4:
			print ("The target's speed is larger than the max speed of robot! ")
		else:
			vel_r = 0.4 #m/s
		print("vel_t:---------------------------------------->",vel_t)
		print("vel_r:---------------------------------------->",vel_r)
		#结果为x和y方向的移动速度，这里可以联立一个一元二次方程，解出两个值。
		dr_distance_x = vel_t * cos_d_l - vel_r * cos_l
		dr_distance_y = vel_t * sin_d_l - vel_r * sin_l
		# distance_x = dif_x - dr_distance_x * time
		# distance_y = dif_y - dr_distance_y * time

		#after we get the deta x and y, we can publish the command to move
		#to the location in the world.
		# if angular_vel == 0:
		# 	self.velocity.twist.angular.z = 0  #math.radians(1)
		# else:
		# 	self.velocity.twist.angular.z = angular_vel
		self.velocity.twist.linear.x = vel_r
		self.velocity.twist.angular.z = angular_vel
		while time < 2:
			# # publish the speed 2 seconds
			# self.velocity_pub.publish(self.velocity)
			break
			time += 0.01
		return vel_r, cos_l,sin_l

	#push the robot to the point in the head
	def push_to_point(self, point):
		push_msg = miro.msg.push()
		push_loc = self.fovea_HEAD 
		push_msg.pushpos = Vector3(push_loc[0], push_loc[1], push_loc[2])
		push_vec = point - push_loc
		push_vec /= np.linalg.norm(push_vec)
		push_vec *= 0.1
		push_msg.pushvec = Vector3(push_vec[0], push_vec[1], push_vec[2])
		push_msg.flags = miro.constants.PUSH_FLAG_IMPULSE
		push_msg.link = miro.constants.LINK_HEAD
		time.sleep(0.1)
		self.push_pub.publish(push_msg)

	def set_forward_speed(self, x):
		x = np.clip(x, -miro.constants.WHEEL_MAX_SPEED_M_PER_S, miro.constants.WHEEL_MAX_SPEED_M_PER_S)
		self.velocity.twist.linear.x = x


	def set_turn_speed(self, z_deg):
		z_rad = math.radians(z_deg)
		z_rad = np.clip(z_rad, -miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S, miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S)
		self.velocity.twist.angular.z = z_rad

	def update_pose(self, msg):

		# get robot feedback, we need to evaluate current pose of the robot, 
		#then we can refer to this for path planning.
		#print ("msg",msg)
		
		#print(spd)
		kin = np.array(msg.kinematic_joints.position)
		#print(kin)
		
		#设定当前姿态的转换，根据机器人的线速度和角速度，姿态包括了 当前物体的坐标和偏航角，
		#在odom的话题中可以提取twist 和 position orientation
		# integrate speed to estimate position
		T = 0.02 
		twist = msg.odom.twist.twist
		dr = twist.linear.x
		dtheta = twist.angular.z
		self.pose[2] += dtheta * T
		#将坐标进行旋转变换，一段时间之后，有了
		dxy = miro.lib.kc.kc_rotate(np.array([dr, 0.0, 0.0]), 'z', self.pose[2])
		self.pose += [dxy[0], dxy[1], 0.0]


	def callback_kin(self,msg):
		self.kinematic_joints =np.array(msg.position)

	def callback_sensors(self,msg):
		self.sensors_pack = msg
