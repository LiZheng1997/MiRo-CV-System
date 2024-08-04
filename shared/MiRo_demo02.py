# -*- coding: utf-8 -*-
#!/usr/bin/python
import miro2 as miro
import rospy
import time
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

# Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

#这个demo可以用来检测相机是否可以用，帧数是否稳定，不稳定的情况下，图像会有延迟。
#最简单的demo 打开两个摄像头 ，获得机器人摄像头的图片信息
#定阅已经存在的相机的话题，使用opencv函数进行展示。

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
			# state
			# hsvl = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)
			# hsvr = cv2.cvtColor(self.cam_right_image, cv2.COLOR_BGR2HSV)
			time.sleep(1)
			outl = self.cam_left_image.copy()
			
			time.sleep(0.1)
			outr = self.cam_right_image.copy()

			cv2.imshow("left",outl )
			cv2.imshow("right",outr )
			if cv2.waitKey(1)&0xff == 27:    # esc键
				cv2.imwrite("left.jpg",outl)
				cv2.imwrite("right.jpg",outr)
				break
		cv2.destroyAllWindows()
		# return none

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

	#take photos of the ROI, and write them into the folder
	def shot(self,pos, frame,flag):
		# global counter
		if flag:
			path = self.folder+"/" + pos + "_" + str(self.counter_left) + ".jpg"
		elif flag != True:
			path = self.folder+"/" + pos + "_" + str(self.counter_right) + ".jpg"
		cv2.imwrite(path, frame)
		print("snapshot saved into: " + path)


if __name__ == "__main__":
	rospy.init_node("demo02", anonymous=True)
	main = Controller("miro")
	main.loop()