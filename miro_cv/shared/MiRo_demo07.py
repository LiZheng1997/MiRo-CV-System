# -*- coding: utf-8 -*-
#!/usr/bin/python
## MIRO进行行人检测的部分样例代码。使用HOG + SVM进行行人检测，分类，这个精度估计不是很高
#后期需要进行negative 的样本的调整。



import miro2 as miro
import rospy
import time
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image
import numpy as np
# Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

#这个demo中主要是套用，MIRO中已经定义好的函数模块进行球相对MIRO的世界坐标的计算
# 这里的坐标给的是（x,y,z)，这个坐标是否是真实环境中的位置信息，还要考证。

class Controller():

	def __init__(self,name):

		self.topic_root = '/' + name + '/'
		# Subscribe to sensors
		#self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage, self.cam_left_callback)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback)

		self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)
		# Arrays to hold image topics
		self.cam_left_image = None
		self.cam_right_image = None

		# Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		# Create resource for controlling body_node
		self.pars = miro.utils.PlatformPars()
		self.cam_model = miro.utils.CameraModel()
		self.frame_w = 0
		self.frame_h = 0

	def detect_pedestrian(self):
		hog = cv2.HOGDescriptor()
		hog.load('myHogDector.bin')
		
		while True:
			time.sleep(0.2)
			image = self.cam_left_image.copy()
			rects, wei = hog.detectMultiScale(image, winStride=(4, 4),padding=(8, 8), scale=1.05)
			for (x, y, w, h) in rects:
				cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
				#在这里进行像素坐标的转换 转换到世界坐标中，我们要的是这个物体在MIRO的相对空间中
				#的距离和速度的信息，这里可以检测到人体之后，或得到的是人体的图框，
			cv2.imshow('a', image)
			if cv2.waitKey(1)&0xff == 27:    # esc键
				break
		cv2.destroyAllWindows()


	def cam_left_callback(self, ros_image):
		try:
			self.cam_left_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
			im_h, im_w = self.cam_left_image.shape[:2]
			if self.frame_w != im_w and self.frame_h != im_h:
				self.frame_w, self.frame_h = im_w, im_h
				self.cam_model.set_frame_size(self.frame_w, self.frame_h)
		except CvBridgeError as e:
			print("Conversion of left image failed \n")
			print(e)

	def cam_right_callback(self, ros_image):
		try:
			self.cam_right_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
			im_h, im_w = self.cam_right_image.shape[:2]
			if self.frame_w != im_w and self.frame_h != im_h:
				self.frame_w, self.frame_h = im_w, im_h
				self.cam_model.set_frame_size(self.frame_w, self.frame_h)
		except CvBridgeError as e:
			print("Conversion of right image failed \n")
			print(e)


if __name__ == "__main__":
	#初始化一个节点 
	rospy.init_node("demo04", anonymous=True)
	main = Controller("miro")
	main.detect_pedestrian()
