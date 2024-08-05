# -*- coding: utf-8 -*-
#!/usr/bin/python
import numpy as np
import copy
import time
import rospy
import cv2
from cv_bridge import CvBridge
import miro2 as miro
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

#这个demo为了检测物体的移动，识别到物体的位移和速度信息。
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
		self.pars = miro.lib.PlatformPars()
		self.cam_model = miro.lib.CameraModel()
		self.frame_w = 0
		self.frame_h = 0

	def loop(self):
		
		time.sleep(1)
		outputl = self.cam_left_image.copy()
		camera = miro.constants.CAM_L


		# ShiTomasi 角点检测参数
		feature_params = dict( maxCorners = 100,
							   qualityLevel = 0.3,
							   minDistance = 7,
							   blockSize = 7 )

		# lucas kanade光流法参数
		lk_params = dict( winSize  = (15,15),
						  maxLevel = 2,
						  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

		# 创建随机颜色
		color = np.random.randint(0,255,(100,3))

		# 获取第一帧，找到角点
		old_frame = outputl
		#找到原始灰度图
		old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

		#获取图像中的角点，返回到p0中
		p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

		# 创建一个蒙版用来画轨迹
		mask = np.zeros_like(old_frame)

		while True:
			time.sleep(0.1)
			frame = self.cam_left_image.copy()
			frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# 计算光流
			p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
			# 选取好的跟踪点
			good_new = p1[st==1]
			good_old = p0[st==1]

			# 画出轨迹
			for i,(new,old) in enumerate(zip(good_new,good_old)):
				a,b = new.ravel()
				c,d = old.ravel()
				mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
				frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
			img = cv2.add(frame,mask)

			cv2.imshow('frame',img)
			k = cv2.waitKey(30) & 0xff
			if k == 27:
				break

			# 更新上一帧的图像和追踪点
			old_gray = frame_gray.copy()
			p0 = good_new.reshape(-1,1,2)
			#print(str(j)+":", p0)

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
	rospy.init_node("demo05", anonymous=True)
	main = Controller("miro")
	main.loop()