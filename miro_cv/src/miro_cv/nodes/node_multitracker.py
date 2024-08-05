import miro2 as miro
import numpy as np
import cv2 as cv
import sys
import rospy
import time
import miro_cv.nodes.node_path_planning

from random import randint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from miro_cv.nodes.node_direction_keeper import *
from geometry_msgs.msg import Pose2D,Vector3

#This class provide a multitracking method, we have three targets for this project's aim
#pedestrian, ball, MIRO, in this case, we can use different detectors to initiate the bounding
#boxes of different objects.
class MultiTracker:

	def __init__(self,name):
		self.topic_root =  name
		self.cam_model = miro.lib.CameraModel()
		#Using the default size of camera
		self.frame_w = 0
		self.frame_h = 0

		# Arrays to hold image topics
		self.cam_left_image = None
		self.cam_right_image = None
		# Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()


		#suscriber
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage,  self.cam_left_callback,queue_size=1,buff_size=52428800)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback,queue_size=1,buff_size=52428800)
		self.pos_sub =  rospy.Subscriber(
			self.topic_root + "sensors/body_pose", Pose2D, self.pose_callback,queue_size=1,buff_size=52428800)

	#call back function for left camera
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

	#call back function for right camera
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
	def pose_callback(self,msg):
		self.pose = msg


	def multitracking_motion(self,bbox_list,cam_index):

		# Set up tracker.
		tracker = cv.MultiTracker_create()
		init_once = False
		#use a counter to calculate the overall frames have been updated
		frame_count = 0
		center_pos = None
		object_x = None
		angle = 0
		old_t_pos = None
		t_move_distance = 0
		old_time = 0
		t_pos = None
		now = 0
		beta = None
		bbox_tuple = ()
		# pose = np.array([self.pose.x,self.pose.y,self.pose.theta])

		# print("self.pose.x-------->:", self.pose)
		if cam_index == 0:
			#get the output of the image from camera
			time.sleep(0.1)
			frame = self.cam_left_image.copy()
		elif cam_index == 1:
			#get the output of the image from camera
			time.sleep(0.1)
			frame = self.cam_right_image.copy()
		if frame is None:
			print("no image is readed")
			pass
		#store three bounding boxes seperately, named pedestrian, miro and ball
		# only ball bounding box has to be checked for the range of the bounding
		# box, other two need not to be calculated.
		for bbox in bbox_list:
			bbox = list(bbox)
			bbox[0] = 0 if bbox[0]< 0 else bbox[0]
			bbox[1] = 0 if bbox[1]< 0 else bbox[1]
			bbox[2] = (640 - bbox[0] -1) if bbox[2] > (640 -bbox[0]) else bbox[2]
			bbox[3] = (360 - bbox[1] -1) if bbox[3] > (360 -bbox[1]) else bbox[3]
			# bbox_list.append(bbox)
			bbox = tuple(bbox)
		if not init_once:
			for bbox in bbox_list:
				p_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox)
			# m_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox2)
			# b_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox3)
		# 	ok = tracker.add(cv.TrackerTLD_create(), frame, bbox1)
		# 	ok = tracker.add(cv.TrackerTLD_create(), frame, bbox2)
		# 	ok = tracker.add(cv.TrackerTLD_create(), frame, bbox3)
			init_once = True	

		while True:
			# print("self.pose.x-------->:", self.pose)
			if cam_index == 0:
				#get the output of the image from camera
				time.sleep(0.1)
				frame = self.cam_left_image.copy()
			elif cam_index == 1:
				#get the output of the image from camera
				time.sleep(0.1)
				frame = self.cam_right_image.copy()
			if frame is None:
				print("no image is readed")
				break
			# Start now, calculate the time.
			now = cv.getTickCount()
			ok, boxes = tracker.update(frame)
			# Calculate Frames per second (FPS)
			fps = cv.getTickFrequency() / (cv.getTickCount() - now)

			for newbox in boxes:
				p1 = (int(newbox[0]), int(newbox[1]))
				p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
				cv.rectangle(frame, p1, p2, (200,0,0))
			#here we have to calculate three distances between those targets and a robot
			#and then give a priority refers to the distance to all targets.
			
			#TODO calculate all distances and calculate the value of priority being chose
			#in this case, we can tracking different targets and select the most dangerous
			# one to intercept.

			
			#set the frames number for this tracking module to update, it means a epoch
			frame_count += 1
			if frame_count == 100:
				cv.destroyAllWindows()
				break

			if cam_index == 0:
				cv.imshow("left_tracking", frame)
			elif cam_index == 1:
				cv.imshow("right_tracking", frame)

			k = cv.waitKey(1) & 0xff
			if k == 27 : break # esc pressed



	def multitracking_motion_s(self,cam_index):

		# Set up tracker.
		tracker = cv.MultiTracker_create()
		## Select boxes
		bboxes = []
		colors = []
		#store three bounding boxes seperately, named pedestrian, miro and ball
		# only ball bounding box has to be checked for the range of the bounding
		# box, other two need not to be calculated.
		# bbox1 = bbox_list[0]
		# bbox2 = bbox_list[1]
		#ball's bounding box

		#use a counter to calculate the overall frames have been updated
		frame_count = 0
		center_pos = None
		object_x = None
		angle = 0
		old_t_pos = None
		t_move_distance = 0
		old_time = 0
		t_pos = None
		now = 0
		beta = None
		# pose = np.array([self.pose.x,self.pose.y,self.pose.theta])

		if cam_index == 0:
			#get the output of the image from camera
			time.sleep(0.1)
			frame = self.cam_left_image.copy()
		elif cam_index == 1:
			#get the output of the image from camera
			time.sleep(0.1)
			frame = self.cam_right_image.copy()
		while True:	
			bbox = cv.selectROI('MultiTracker', frame)
			bboxes.append(bbox)
			print("Press q to quit selecting boxes and start tracking")
			# print("Press any other key to select next object")
			colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))
			k = cv.waitKey(0) & 0xFF
			if (k == 113):  # q is pressed
				cv.destroyAllWindows()
				break
		for bbox in bboxes:
			p_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox)
			m_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox)
			b_ok = tracker.add(cv.TrackerTLD_create(), frame, bbox)

		while True:
			# print("self.pose.x-------->:", self.pose)
			if cam_index == 0:
				#get the output of the image from camera
				time.sleep(0.1)
				frame = self.cam_left_image.copy()
			elif cam_index == 1:
				#get the output of the image from camera
				time.sleep(0.1)
				frame = self.cam_right_image.copy()
			if frame is None:
				print("no image is readed") 
				break

			# Start now, calculate the time.
			now = cv.getTickCount()
			ok, boxes = tracker.update(frame)
			# Calculate Frames per second (FPS)
			fps = cv.getTickFrequency() / (cv.getTickCount() - now)
			for i, newbox in enumerate(boxes):
				p1 = (int(newbox[0]), int(newbox[1]))
				p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
				# cv.rectangle(frame, p1, p2,colors[i], 2, 1)
				cv.rectangle(frame, p1, p2, 2, 1)
				#here we have to calculate three distances between those targets and a robot
				#and then give a priority refers to the distance to all targets.
				

				#I set the gain of each class of targets as 0.8, 0.5, 0.2, the target, whose distance
				# is less than 1 meter between a robot, will get a value of priority. 
				# Priorities are 0.25-->[1 meter,0.75 m) 0.5-->[0.75, 0.5) 0.75-->[0.5,0.25) 1-->[0.25,0.1)
				# Our sonor safety control will set a saftey distance as 0.1, if the distance is smaller
				# than 0.1 meter, the interception is suscessful.

				#TODO calculate all distances and calculate the value of priority being chose
				#in this case, we can tracking different targets and select the most dangerous
				# one to intercept.

				
				#set the frames number for this tracking module to update, it means a epoch
				# frame_count += 1
				# if frame_count == 100:
				# 	cv.destroyAllWindows()
				# 	break

			if cam_index == 0:
				cv.imshow("left_tracking", frame)
			elif cam_index == 1:
				cv.imshow("right_tracking", frame)

			k = cv.waitKey(1) & 0xff
			if k == 27 : break # esc pressed