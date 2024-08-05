# -*- coding: utf-8 -*-
#!/usr/bin/python
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


class Controller():

	def __init__(self,name):

		self.topic_root = '/' + name + '/'
		# Subscribe to sensors
		#self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage, self.cam_left_callback)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback)

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
		print("init")

	def loop(self):

		while True:
			#记得加上一个sleep的时间，不然启动节点收到信息 比较慢。让他缓存数据
			time.sleep(0.1)
			# state
			hsvl = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)
			#hsvr = cv2.cvtColor(self.cam_right_image, cv2.COLOR_BGR2HSV)
			# get blue mask
			# target_hue = hsvl[0,0][0]
			# lo = np.array([target_hue-20, 70, 70])
			# hi = np.array([target_hue+20, 255, 255])
			# mask = cv2.inRange(hsvl, lo, hi)
		
			time.sleep(1)
			outl = self.cam_left_image.copy()
			lo = np.array([80,  40,  40])
			hi = np.array([140, 255, 255])
			mask = cv2.inRange(hsvl, lo, hi)

			# clean up
			seg = mask
			seg = cv2.GaussianBlur(seg, (5, 5), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)

			#使用霍夫圆函数算法进行圆形检测
			# get circles
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT,
					1, 40, param1=10, param2=33, minRadius=0, maxRadius=40)

			# get largest circle 
			max_circle = None
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))

				for c in circles[0,:]:
					cv2.circle(outl, (c[0], c[1]), c[2], (0, 255, 0), 2)

					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c

			#time.sleep(0.1)
			cv2.imshow("seg", outl)
			# time.sleep(1)
			# cv2.imshow("right", hsvr)
			if cv2.waitKey(1)&0xff == 27:    # esc键
				break
		print(max_circle)
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
	main.loop()

