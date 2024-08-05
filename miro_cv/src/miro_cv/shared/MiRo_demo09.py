# -*- coding: utf-8 -*-
#!/usr/bin/python
#当前demo进行MIRO的移动，移动到指定的位置，完成截取任务。
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

#kc的一些基本值的初始化，
# function to initialise default values
def kc_init(tick, flags, link):
	global KC_TICK_SEC, KC_PUSH_FLAGS_DEFAULT, KC_PUSH_LINK_DEFAULT
	KC_TICK_SEC=tick
	KC_PUSH_FLAGS_DEFAULT=flags 
	KC_PUSH_LINK_DEFAULT=link

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

		kc = miro.utils.kc_interf.kc_miro()
		# create objects in HEAD
		pos = miro.utils.get("LOC_NOSE_TIP_HEAD")
		vec = np.array([1.0, 0.0, 0.0])

		# transform to WORLD (note use of "Abs" and "Rel"
		# for positions and directions, respectively)
		posw = kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pos)
		vecw = kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, vec)
		# self.flags = KC_PUSH_FLAGS_DEFAULT
		# self.link = KC_PUSH_LINK_DEFAULT
		# self.pos = np.array([0.0, 0.0, 0.0])
		# self.vec = np.array([0.0, 0.0, 0.0])
		#
		# self.clock = ActionClock()

		self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)

	def move_to_target(self):
		#获得目标的位置，这里给个球的位置的。
		while True:
			loc = self.get_ball_loc()
			self.push_to_point(loc)
		
	def get_ball_loc(self):

		#while True:
		time.sleep(1)
		hsvl = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)

		#extract boundaries for masking image
		lo = np.array([80,  40,  40])
		hi = np.array([140, 255, 255])
		mask = cv2.inRange(hsvl, lo, hi)

		# Search for largest circle in either frame
		largest_circle = None
		circle_loc = None
		circle_cam = None
		largest_radius = 0


		outputl = self.cam_left_image.copy()
		camera = miro.constants.CAM_L

		# time.sleep(0.1)
		# hsv_image = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)
		#mask image
		#mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
		seg = mask

		# Do some processing
		seg = cv2.GaussianBlur(seg, (11,11), 0)
		seg = cv2.erode(seg, None, iterations=2)
		seg = cv2.dilate(seg, None, iterations=2)

		time.sleep(0.1)
		# get circles
		circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=33, minRadius=0, maxRadius=0)

		#Search through circles for largest
		if circles is not None:
			circles = np.uint16(np.around(circles))
			for circle in circles[0,:]:
				if circle[2] > largest_radius:
					largest_radius = circle[2]
					largest_circle = circle
					circle_cam = camera
		else:
			return None
		#这几句模板直接用，计算出球的世界坐标的位置，
		circle_loc = [largest_circle[0],largest_circle[1]]
		view_line = self.cam_model.p2v(circle_loc)
		circle_point = self.cam_model.v2oh(circle_cam, view_line, 1.0)
		print(circle_point)
		print(circle_loc)

		#time.sleep(0.1)
		#这里用来调试图像，找球的位置。
		#Debug Only
		cv2.circle(outputl, (largest_circle[0], largest_circle[1]), largest_circle[2], (0, 255, 0), 2)
		# location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
		# text_y_offset = 18
		# for i, line in enumerate(location_str.split(",")):
		# 	text_y = max_circle[1] - text_y_offset + i*text_y_offset
		# 	cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
		# 	cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
		#time.sleep(0.1)
		#cv2.imshow("out",outputl)
		if cv2.waitKey(1)&0xff == 27:    # esc键
			pass
		cv2.destroyAllWindows()

		return circle_point

	def push_to_point(self, point):
		#self.pause()
		push_msg = miro.msg.push()
		push_loc = miro.utils.get("LOC_SONAR_FOVEA_HEAD")
		push_msg.pushpos = Vector3(push_loc[0], push_loc[1], push_loc[2])
		push_vec = point - push_loc
		push_vec /= np.linalg.norm(push_vec)
		push_vec *= 0.1
		push_msg.pushvec = Vector3(push_vec[0], push_vec[1], push_vec[2])
		push_msg.flags = miro.constants.PUSH_FLAG_IMPULSE
		push_msg.link = miro.constants.LINK_HEAD
		self.push_pub.publish(push_msg)

	def pixel_to_point(self, pixel_loc, range, cam_index):
		self.pause()
		if pixel_loc != None:
			view_line = self.cam_model.p2v(pixel_loc) #Transform to view line in CAM
			point_head = self.cam_model.v2oh(cam_index, view_line, range)
			return point_head
		else:
			return None


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
	rospy.init_node("demo09", anonymous=True)
	main = Controller("miro")
	main.move_to_target()