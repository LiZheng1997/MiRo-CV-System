# -*- coding: utf-8 -*-
#!/usr/bin/python
#This node is designed for processing images, all image data will be processed through
#using some OpenCV algorithms, then all processed data will be published on a 
#topic named xxxxxx

import miro2 as miro
import rospy
import time
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

# Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from camera_callback import *

class Image_process:
	

	def __init__(self,name):
		
		#get the robot name
		self.topic_root =  name
		self.camera = Camera(self.topic_root)


		# # Arrays to hold image topics
		# self.cam_left_image = None
		# self.cam_right_image = None

		# Create object to convert ROS images to OpenCV format
		#self.image_converter = CvBridge()

		# Subscribe to sensors
		#self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage,  self.camera.cam_left_callback)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.camera.cam_right_callback)

	#这里到时候可以加上一个，用户自己选颜色范围的参数，因为球的颜色不同可以自己自定义16进制的颜色范围	
	def image_processing(self):
		
		# transform the image to the HSV format
		time.sleep(1)
		hsvl = cv2.cvtColor(self.camera.cam_left_image, cv2.COLOR_BGR2HSV)
		time.sleep(1)
		hsvr = cv2.cvtColor(self.camera.cam_right_image, cv2.COLOR_BGR2HSV)
		return hsvl, hsvr
			#没有格式进行转换 HSV格式不能被解码。
			# hsvl = self.image_converter.cv2_to_compressed_imgmsg(hsvl,"hsv")
			# hsvr = self.image_converter.cv2_to_compressed_imgmsg(hsvr,"hsv")
			# #Publish the processed images 
			# self.processed_imager.publish(hsvr)
			# self.processed_imagel.publish(hsvl)
