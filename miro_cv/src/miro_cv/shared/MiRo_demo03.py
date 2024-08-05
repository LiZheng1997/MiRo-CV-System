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
import numpy as np
from copy import copy

#这个demo实现了在相机的图中找到圆，使用霍夫圆函数进行查询。
#这里，使用了两个

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
			time.sleep(0.1)
			# b,g,r = cv2.split(self.cam_left_image)
			# bgr_colour = np.uint8([[[b,g,r]]])

			hsvl = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)

			im_h = np.size(hsvl, 0)
			im_w = np.size(hsvl, 1)
			im_centre_h = im_h / 2.0
			im_centre_w = im_w / 2.0

		
			time.sleep(0.1)
			outputl = self.cam_left_image.copy()
			time.sleep(0.1)
			outputr = self.cam_right_image.copy()

			cv2.line(outputl, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
			cv2.line(outputl, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
			
			
			#extract boundaries for masking image
			target_hue = hsvl[0,0][0]
			lower_bound = np.array([target_hue-20, 70, 70])
			upper_bound = np.array([target_hue+20, 255, 255])

			#mask image
			mask = cv2.inRange(hsvl, lower_bound, upper_bound)
			seg = mask

			# Do some processing
			seg = cv2.GaussianBlur(seg, (11,11), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)

			# get circles
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=20,minRadius=0, maxRadius=0)

			# Get largest circle
			max_circle = None
			max_circle_norm = [None, None, None]
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))

				for c in circles[0,:]:
					cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)

					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c
						max_circle_norm[0] = int(round(((max_circle[0] - im_centre_w) / im_centre_w) * 100.0))
						max_circle_norm[1] = int(round(-((max_circle[1] - im_centre_h) / im_centre_h) * 100.0))
						max_circle_norm[2] = int(round((max_circle[2]/im_centre_w)*100.0))

					#Debug Only
					cv2.circle(outputl, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
					cv2.circle(outputl, (max_circle[0], max_circle[1]), 1, (0, 255, 0), 2)
					location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
					text_y_offset = 18
					for i, line in enumerate(location_str.split(",")):
						text_y = max_circle[1] - text_y_offset + i*text_y_offset
						cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
						cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

			else:
				pass

			#debug_image = self.image_converter.cv2_to_imgmsg(outputl, "bgr8")
			time.sleep(0.1)
			cv2.imshow("left", outputl)
			#time.sleep(1)
			#cv2.imshow("debug_image", debug_image)
			print(max_circle_norm)

			# cv2.imshow("left", hsvl)
			# time.sleep(1)
			# hsvr = cv2.cvtColor(self.cam_right_image, cv2.COLOR_BGR2HSV)
			# cv2.imshow("right", hsvr)
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
	rospy.init_node("demo03", anonymous=True)
	main = Controller("miro")
	main.loop()

