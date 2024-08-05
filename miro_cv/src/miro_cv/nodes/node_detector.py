# -*- coding: utf-8 -*-
#!/usr/bin/python
#This is the module for detecting all targets, different functions based on various algorithms to 
#implement a detection module.
## MIRO进行行人检测的部分样例代码。使用HOG + SVM进行行人检测，分类，这个精度估计不是很高
#后期需要进行negative 的样本的调整。


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


"""Using normal OpenCV APIs to implement detection modules, so I call it Normal detector, and there is another
modeule called CNN detector, this module will use neural network models to implement detections."""

class NormalDetector:

	def __init__ (self,name, cascade_path):
		self.topic_root = name
		self.cascade_path = cascade_path
		#set the default size of the camera
		self.cam_model = miro.lib.CameraModel()
		self.frame_w = 0
		self.frame_h = 0
		# self.cam_model.set_frame_size(640, 320)
		
		# Subscribe to sensors
		#self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.counter_left = 0 
		self.counter_right = 0
		self.pedestrian_pos = []
		#这里记得把地址搞成相对地址，之前试过应该是没问题的
		self.folder = "../images"
		# Arrays to hold image topics
		self.cam_left_image = None
		self.cam_right_image = None
		# Create objects to convert ROS images to OpenCV format
		self.image_converter = CvBridge()
		# self.transformer = Transformer(name)
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage, self.cam_left_callback,queue_size=1,buff_size=52428800)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback,queue_size=1,buff_size=52428800)

	#call back function for the left camera
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

	#call back function for the right camera
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

	def detect_MIRO_l(self):
		count =0 
		#The cascade xml file can be changed in later stage, more accurate 
		#model is mandatory
		bbox_lst_l = []
		miros_l = []
		bbox = () #bounding box is a tuple format
		miroCascade = cv2.CascadeClassifier(self.cascade_path)# 这里用绝对路径
		while True:
			time.sleep(0.1)
			outputl = self.cam_left_image.copy()
			# time.sleep(0.1)
			gray_l = cv2.cvtColor(outputl, cv2.COLOR_BGR2GRAY)
			im_h = np.size(gray_l, 0)
			im_w = np.size(gray_l, 1)
			im_centre_h = im_h / 2.0
			im_centre_w = im_w / 2.0
			# draw the calibration lines at x and y axis in the middle of the image.
			cv2.line(outputl, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
			cv2.line(outputl, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
			miros_l = miroCascade.detectMultiScale(gray_l,scaleFactor = 1.15,minNeighbors = 5,minSize = (1,1))  #minSize = (2,2)
			if miros_l is not None:
				miros_l = np.uint16(np.around(miros_l))
				for(x,y,w,h) in miros_l:
					circle_loc = [x + w/2, y + h/2]
					bbox = (x,y,w,h)
					# print bbox
					cv2.rectangle(outputl, (x, y), (x + w, y + h), (0, 0, 255), 2)
			else:
				print("I cannot find any MiRo robots on the left side!")
				pass
			bbox_lst_l.append(bbox)
			#For testing only
			# cv2.imshow("left_tracking", outputl)
			# cv2.imshow("gray_l", gray_l)
			# if cv2.waitKey(0)&0xff == 27:
			# 	cv2.destroyAllWindows()
			# 	break
			count += 1  
			if count == 5:
				count = 0
				break
		cv2.destroyAllWindows()
		return  bbox_lst_l , outputl

	def detect_MIRO_r(self):
		count =0
		bbox_lst_r = []
		miros_r = []
		bbox = () #bounding box is a tuple format
		miroCascade = cv2.CascadeClassifier(self.cascade_path)
		# cv2.namedWindow("detect_miros_right")
		while True:
			time.sleep(0.1)
			outputr = self.cam_right_image.copy()
			# time.sleep(0.1)
			gray_r = cv2.cvtColor(outputr, cv2.COLOR_BGR2GRAY)

			# gray_r = cv2.GaussianBlur(gray_r, (11, 11), 0)
			# gray_r = cv2.erode(gray_r, None, iterations=2)
			# gray_r = cv2.dilate(gray_r, None, iterations=2)
			# gray_r = cv2.equalizeHist(gray_r)

			im_h = np.size(gray_r, 0)
			im_w = np.size(gray_r, 1)
			im_centre_h = im_h / 2.0
			im_centre_w = im_w / 2.0
			# draw the calibration lines at x and y axis in the middle of the image.
			cv2.line(outputr, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
			cv2.line(outputr, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
			miros_r = miroCascade.detectMultiScale(gray_r,scaleFactor = 1.15,minNeighbors = 5,minSize = (1,1)) #,flags=cv2.CASCADE_SCALE_IMAGE #,minSize = (2,2)
			if miros_r is not None:
				miros_r = np.uint16(np.around(miros_r))
				for(x,y,w,h) in miros_r:
					circle_loc = [x + w/2, y + h/2]
					bbox = (x,y,w,h)
					cv2.rectangle(outputr, (x, y), (x + w, y + h), (0, 0, 255), 2)
			else:
				print("I cannot find any MiRo robots on the right side!")
				pass
			bbox_lst_r.append(bbox)
			# cv2.imshow("right_tracking", outputr)
			# if cv2.waitKey(1)&0xff == 27:
			# 	cv2.destroyAllWindows()	    # esc键
			# 	break
			count += 1  
			if count == 5:
				count = 0
				break
		cv2.destroyAllWindows()	
		return  bbox_lst_r, outputr

	#This is the function which is aimed at detecting the pedestrian. The algorithm I used here is HOG and SVM.
	def detect_pedestrian(self):
		count = 0
		pedestrian_pos_l = []
		ROI_l = []
		pedestrian_pos_r = []
		ROI_r = []
		hog = cv2.HOGDescriptor()
		#load the trained bin file
		hog.load('../bin/pedestrian_bin/myHogDector.bin')
		# pedestrian_cascade = cv2.CascadeClassifier('/home/lizheng/Documents/MIRO_demos/Demo/bin/myHogDector.bin')
		while True:
			if self.cam_right_callback != None:
				time.sleep(0.1)
				outputr = self.cam_right_image.copy()
			if self.cam_left_callback != None:
				time.sleep(0.1)
				outputl = self.cam_left_image.copy()
			rects_l, wei = hog.detectMultiScale(outputl, winStride=(4, 4),padding=(8, 8), scale=1.05)
			rects_r, wei = hog.detectMultiScale(outputr, winStride=(4, 4),padding=(8, 8), scale=1.05)
			# rects_l = pedestrian_cascade.detectMultiScale(outputr, winStride=(4, 4),padding=(8, 8), scale=1.05)
			# rects_r = pedestrian_cascade.detectMultiScale(outputl, winStride=(4, 4),padding=(8, 8), scale=1.05)
			for (x, y, w, h) in rects_l:
				#get the position of the human's foot
				pedestrian_pos_l = [x + w/2, y + h]
				#viewing the bounding boxes
				cv2.rectangle(outputl, (x, y), (x + w, y + h), (0, 0, 255), 2)
				#在这里进行像素坐标的转换 转换到世界坐标中，我们要的是这个物体在MIRO的相对空间中
				#的距离和速度的信息，这里可以检测到人体之后，或得到的是人体的图框，
				#save all ROI , and then we will form the negative samples, 
				ROI_l = outputl[y:y+h,x:x+w]
				#after use the shot function ,we only want a accurate ROI to submit to 
				#the tracker functions. uncomment below code to get the exact bounding box.
				#one problem now have not been solved, we want some exact bounding boxes for
				#pedestrians, the accuracy is a import causes for avoiding unexpected bounding boxes.
				# ROI = outputl[y:y+h,x:x+w]
				# if count %2 == 0:
				# 	self.shot("left",ROI_l,True)
				# 	self.counter_left+= 1
				# 	print(self.counter_left)
				pedestrian_pos_l.append(pedestrian_pos_l)
			# cv2.imshow('left', outputl)

			for (x, y, w, h) in rects_r:
				#get the position of the human's foot
				pedestrian_pos_r = [x + w/2, y + h]
				#viewing the bounding boxes
				cv2.rectangle(outputr, (x, y), (x + w, y + h), (0, 0, 255), 2)
				ROI_r = outputr[y:y+h,x:x+w]
				# if count %2 == 0:
				# 	self.shot("right",ROI_r,False)
				# 	self.counter_right+= 1
				# 	print(self.counter_right)
				pedestrian_pos_r.append(pedestrian_pos_r)
			# cv2.imshow('right', outputr)

		# 	count += 1
		# 	if cv2.waitKey(1)&0xff == 27:    # esc键
		# 		break
		# cv2.destroyAllWindows()
		# return pedestrian_pos_l, ROI_l,pedestrian_pos_r,ROI_r

	#take photos of the ROI, and write them into the folder
	def shot(self,pos, frame,flag):
		# global counter
		if flag:
			path = self.folder+"/" + pos + "_" + str(self.counter_left) + ".jpg"
		elif flag != True:
			path = self.folder+"/" + pos + "_" + str(self.counter_right) + ".jpg"
		cv2.imwrite(path, frame)
		print("snapshot saved into: " + path)

	def detect_balls_l(self,colour_str):
		count = 0
		# circle_pos_l = np.array([],dtype=UInt16)
		# ball_pos_l = np.array([],dtype=UInt16)
		bbox_lst_l = []
		# circle_loc = None
		# get largest circle , take out the largest ball
		max_circle = None
		# circle_point = None
		bbox = () #bounding box is a tuple format
		max_circle_norm = [None, None, None]
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return

		if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
			return
		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lower_bound = np.array([target_hue-20, 100, 100])
		upper_bound = np.array([target_hue+20, 255, 255])
		
		while True:
			#get a copy of the raw image without being processed.
			time.sleep(0.1)
			outputl = self.cam_left_image.copy()
			#here is the range of the hsv paramaters we want to pick 
			# time.sleep(0.1)
			hsvl = cv2.cvtColor(outputl, cv2.COLOR_BGR2HSV)
			#get the image width and height in HSV format, and 
			#the center coordination of the image. we can assume that 
			# the attribute of left image and the right is the same.
			im_h = np.size(hsvl, 0)
			im_w = np.size(hsvl, 1)
			im_centre_h = im_h / 2.0
			im_centre_w = im_w / 2.0
			#draw the calibration lines at x and y axis in the middle of the image.
			cv2.line(outputl, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
			cv2.line(outputl, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
			# lower_bound = np.array([110,  100,  100])
			# upper_bound = np.array([130, 255, 255])
			#mask image, get a mask of the left hsvl image. we have set the bounds.
			mask = cv2.inRange(hsvl, lower_bound, upper_bound)
			seg = mask # here is the segmentation of the image
			# Do some processing
			seg = cv2.GaussianBlur(seg, (11,11), 0)
			# seg = cv2.GaussianBlur(seg, (5,5), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)
			# get circles, the set parameter in the function can be changed, due to different balls
			#when you want to enhance the accuracy, you can adjust the parameter.
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=20,minRadius=0, maxRadius=0)
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))
				for c in circles[0,:]:
					#draw the circle, set the position and the radius of the circle.
					cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)
					#judge if the radius of the ball is over the max_rad, then pick out 
					#the largest one.
					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c
						# circle_loc = [max_circle[0], max_circle[1]]
						bbox = ((max_circle[0] - max_circle[2]),(max_circle[1] - max_circle[2]),\
									(2*c[2]), (2*c[2]) )	
						#get the normalized cirle attributes
						max_circle_norm[0] = int(round(((max_circle[0] - im_centre_w) / im_centre_w) * 100.0))
						max_circle_norm[1] = int(round(-((max_circle[1] - im_centre_h) / im_centre_h) * 100.0))
						max_circle_norm[2] = int(round((max_circle[2]/im_centre_w)*100.0))
					#Debug Only
					cv2.circle(outputl, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
					cv2.circle(outputl, (max_circle[0], max_circle[1]), 1, (0, 50, 0), 2)
					location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
					text_y_offset = 18
					for i, line in enumerate(location_str.split(",")):
						text_y = max_circle[1] - text_y_offset + i*text_y_offset
						cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
						cv2.putText(outputl, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
			else:
				print ("No balls have been found on the left side.")
				pass
			# cv2.imshow("outl", outputl)
			# circle_point = self.pixel_to_point(circle_loc, 1.0, miro.constants.CAM_L )
			# circle_pos_l = np.append(circle_pos_l,circle_loc)
			# ball_pos_l = np.append(ball_pos_l,circle_point)
			bbox_lst_l.append(bbox)
			count += 1  
			if count == 3:
				count = 0
				break
		# 	if cv2.waitKey(1)&0xff == 27: # esc键
		# 		break
		# cv2.destroyAllWindows()
		return bbox_lst_l , outputl

	def detect_balls_r(self,colour_str):
		count = 0
		# circle_pos_r = np.array([],dtype=UInt16)
		# ball_pos_r = np.array([],dtype=UInt16)
		bbox_lst_r = []
		# circle_loc = None
		# get largest circle , take out the largest ball
		max_circle = None
		# circle_point = None
		bbox = () #bounding box is a tuple format
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return

		if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
			return

		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lower_bound = np.array([target_hue-20, 100, 100])
		upper_bound = np.array([target_hue+20, 255, 255])
		max_circle_norm = [None, None, None]
		
		while True:
			#get a copy of the raw image without being processed.
			time.sleep(0.1)
			outputr = self.cam_right_image.copy()
			#here is the range of the hsv paramaters we want to pick 
			time.sleep(0.1)
			hsvr = cv2.cvtColor(outputr, cv2.COLOR_BGR2HSV)
			#get the image width and height in HSV format, and 
			#the center coordination of the image. we can assume that 
			# the attribute of left image and the right is the same.
			im_h = np.size(hsvr, 0)
			im_w = np.size(hsvr, 1)
			im_centre_h = im_h / 2.0
			im_centre_w = im_w / 2.0
			#draw the calibration lines at x and y axis in the middle of the image.
			cv2.line(outputr, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
			cv2.line(outputr, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
			# lower_bound = np.array([0,  90,  90])
			# upper_bound = np.array([10, 255, 255])
			#mask image, get a mask of the left hsv image. we have set the bounds.
			mask = cv2.inRange(hsvr, lower_bound, upper_bound)
			seg = mask # here is the segmentation of the image
			# Do some processing
			seg = cv2.GaussianBlur(seg, (11,11), 0)
			# seg = cv2.GaussianBlur(seg, (5,5), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)
			# get circles, the set parameter in the function can be changed, due to different balls
			#when you want to enhance the accuracy, you can adjust the parameter.
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=20,minRadius=0, maxRadius=0)
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))
				for c in circles[0,:]:
					#draw the circle, set the position and the radius of the circle.
					cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)
					#judge if the radius of the ball is over the max_rad, then pick out 
					#the largest one.
					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c
						circle_loc = [max_circle[0], max_circle[1]]
						#here, the bbox is a tuple, frist two are the top left position, others are width and height
						bbox = ((max_circle[0] - max_circle[2]),(max_circle[1] - max_circle[2]),\
									(2*c[2]), (2*c[2]) )
						#get the normalized cirle attributes
						max_circle_norm[0] = int(round(((max_circle[0] - im_centre_w) / im_centre_w) * 100.0))
						max_circle_norm[1] = int(round(-((max_circle[1] - im_centre_h) / im_centre_h) * 100.0))
						max_circle_norm[2] = int(round((max_circle[2]/im_centre_w)*100.0))
					#Debug Only
					cv2.circle(outputr, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
					cv2.circle(outputr, (max_circle[0], max_circle[1]), 1, (0, 50, 0), 2)
					location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
					text_y_offset = 18
					for i, line in enumerate(location_str.split(",")):
						text_y = max_circle[1] - text_y_offset + i*text_y_offset
						cv2.putText(outputr, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
						cv2.putText(outputr, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
			else:
				print ("No balls have been found on the right side.")
				pass
			# cv2.imshow("outr", outputr)
			# ball_loc = self.pixel_to_point(circle_loc, 1.0, 1 )	
			#cv2.imshow("seg", seg)
			# circle_pos_r = np.append(circle_pos_r,circle_loc)
			# ball_pos_r = np.append(ball_pos_r,ball_loc)
			bbox_lst_r.append(bbox)
			count += 1  
			if count == 3:
				count = 0
				break
		# 	if cv2.waitKey(1)&0xff == 27: # esc键
		# 		break
		# cv2.destroyAllWindows()
		return  bbox_lst_r,outputr


	#I define two detection methods, so we can get the data seperately
	def detect_ball_r(self,colour_str):
		count = 0
		# circle_pos_r = np.array([],dtype=UInt16)
		# ball_pos_r = np.array([],dtype=UInt16)
		bbox_lst_r = []
		# circle_loc = None
		# get largest circle , take out the largest ball
		max_circle = None
		# circle_point = None
		bbox = () #bounding box is a tuple format
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return
		if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
			return
		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)
		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lo = np.array([target_hue-20, 70, 70])
		hi = np.array([target_hue+20, 255, 255])

		while True:
			#get a copy of the raw image without being processed.
			time.sleep(0.1)
			outputr = self.cam_right_image.copy()
			#here is the range of the hsv paramaters we want to pick 
			time.sleep(0.1)
			hsvr = cv2.cvtColor(outputr, cv2.COLOR_BGR2HSV)
			# lo = np.array([0,  126,  90])
			# hi = np.array([4, 255, 255])
			mask = cv2.inRange(hsvr, lo, hi)
			# Use some OpenCV methods to take out the noises.
			seg = mask
			seg = cv2.GaussianBlur(seg, (11, 11), 0)
			# seg = cv2.GaussianBlur(seg, (5, 5), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)
			#使用霍夫圆函数算法进行圆形检测
			# get circles, if you want to reguralize the parameters, you have to try 
			#in real world for several different times. In Gazebo, we only detect blue ball.
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT,
					1, 40, param1=10, param2=33,  minRadius=0, maxRadius=0)
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))
				#compare the different circles have been detected and select the largest one.
				for c in circles[0,:]:
					#test only
					cv2.circle(outputr, (c[0], c[1]), c[2], (0, 255, 0), 2)
					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c
						circle_loc = [max_circle[0], max_circle[1]]
						#get the up left and bottom right point of the bounding box
						# p1 = ((max_circle[0] - max_circle[2]), (max_circle[1]-max_circle[2]))
						# p2 = ((max_circle[0] + max_circle[2]), (max_circle[1]+max_circle[2]))
						# cv2.rectangle(output, p1, p2, (255,0,0), 2, 1)
						#here, the bbox is a tuple, frist two are the top left position, others are width and height
						bbox = ((max_circle[0] - max_circle[2]),(max_circle[1] - max_circle[2]),\
									(2*c[2]), (2*c[2]) )
						print("r_max_circle:",circle_loc)
						# circle_point = self.pixel_to_point(circle_loc, 1.0, 1 )	
			else:
				pass
			# cv2.imshow("outr", outputr)
			# cv2.imshow("segr", seg)
			# circle_pos_r = np.append(circle_pos_r,circle_loc)
			# ball_pos_r = np.append(ball_pos_r,circle_point)
			bbox_lst_r.append(bbox)
			count += 1
			if count == 2:
				count =0
				break
			# if cv2.waitKey(0)&0xff == 27: # esc键
			# 	break
		# cv2.destroyAllWindows()
		return  bbox_lst_r , outputr
		
	def detect_ball_l(self,colour_str):
		count =0
		# circle_pos_l = np.array([],dtype=UInt16)
		# ball_pos_l = np.array([],dtype=UInt16)
		bbox_lst_l = []
		# circle_loc = None
		# get largest circle , take out the largest ball
		max_circle = None
		# circle_point = None
		bbox = () #bounding box is a tuple format
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return
		if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
			return
		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)
		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lo = np.array([target_hue-20, 70, 70])
		hi = np.array([target_hue+20, 255, 255])
		while True:
			#get a copy of the raw image without being processed.
			time.sleep(0.1)
			outputl = self.cam_left_image.copy()
			#here is the range of the hsv paramaters we want to pick 
			time.sleep(0.1)
			hsvl = cv2.cvtColor(outputl, cv2.COLOR_BGR2HSV)
			# lo = np.array([0,  111,  61])
			# hi = np.array([5, 255, 255])
			mask = cv2.inRange(hsvl, lo, hi)
			# Use some OpenCV methods to take out the noises.
			seg = mask
			seg = cv2.GaussianBlur(seg, (11, 11), 0)
			# seg = cv2.GaussianBlur(seg, (5, 5), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)
			#使用霍夫圆函数算法进行圆形检测
			# get circles, if you want to reguralize the parameters, you have to try 
			#in real world for several different times. In Gazebo, we only detect blue ball.
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT,
					1, 40, param1=10, param2=33, minRadius=0, maxRadius=0)
			if circles is not None:
				self.max_rad = 0
				circles = np.uint16(np.around(circles))
				#compare the different circles have been detected and select the largest one.
				for c in circles[0,:]:
					#test only
					cv2.circle(outputl, (c[0], c[1]), c[2], (0, 255, 0), 2)
					if c[2] > self.max_rad:
						self.max_rad = c[2]
						max_circle = c
						circle_loc = [max_circle[0], max_circle[1]]
						#get the up left and bottom right point of the bounding box
						# p1 = ((max_circle[0] - max_circle[2]), (max_circle[1]-max_circle[2]))
						# p2 = ((max_circle[0] + max_circle[2]), (max_circle[1]+max_circle[2]))
						# cv2.rectangle(outputl, p1, p2, (255,0,0), 2, 1)
						#here, the bbox is a tuple, frist two are the top left position, others are width and height
						bbox = ((max_circle[0] - max_circle[2]),(max_circle[1] - max_circle[2]),(2*max_circle[2]), (2*max_circle[2]) )
						print("l_max_circle:",circle_loc)
						# circle_point = self.pixel_to_point(circle_loc, 1.0, 0)
			else:
				pass
			# cv2.imshow("outl", outputl)
			# cv2.imshow("segl", seg)
			# print("l_bbox:",bbox)
			# circle_pos_l = np.append(circle_pos_l,circle_loc)
			# ball_pos_l = np.append(ball_pos_l,circle_point)
			bbox_lst_l.append(bbox)
			# print("circle position2", self.circle_pos)
			# print(self.circle_pos.shape)
			#set a counter for looping times
			count += 1
			if count == 2:
				count =0
				break
			# if cv2.waitKey(1)&0xff == 27: # esc键
			# 	break
		# cv2.destroyAllWindows()
		return bbox_lst_l , outputl


	#This method use a differnet method to detect the movement frames by frame,
	#the result contains all pixels' movements.
	def detect_movement_info(self):
		time.sleep(0.1)
		outputr = self.cam_right_image.copy()
		time.sleep(0.1)
		outputl = self.cam_left_image.copy()
		prvs = cv2.cvtColor(outputl,cv2.COLOR_BGR2GRAY)
		hsv = np.zeros_like(outputl)

		#遍历每一行的第1列
		hsv[...,1] = 255

		while(1):
			time.sleep(1)
			outputl = self.cam_left_image.copy()
			next = cv2.cvtColor(outputl,cv2.COLOR_BGR2GRAY)

			#返回一个两通道的光流向量，实际上是每个点的像素位移值,
			flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)

			#print(flow.shape)
			print(flow)

			#笛卡尔坐标转换为极坐标，获得极轴和极角
			mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
			hsv[...,0] = ang*180/np.pi/2
			hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
			rgb = cv2.cvtCdetect_movement_infoolor(hsv,cv2.COLOR_HSV2BGR)
			print(rgb.shape)
			cv2.imshow('frame2',rgb)
			k = cv2.waitKey(30) & 0xff
			if k == 27:
				break
			#if you want to save a sample of the optical flow and hsv format picture
			elif k == ord('s'):
				cv2.imwrite('opticalfb.png',hsv)
				cv2.imwrite('opticalhsv.png',rgb)
			prvs = next

		cv2.destroyAllWindows()
	
	#This function uses ShiTomasi and  lucas kanade to get some movements in 
	#a frame, in this case, we can limit the detection region.
	def detect_movement(self):

		time.sleep(0.1)
		outputr = self.cam_right_image.copy()
		time.sleep(0.1)
		outputl = self.cam_left_image.copy()

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
		print("p0------->:",p0)
		# 创建一个蒙版用来画轨迹
		mask = np.zeros_like(old_frame)

		while True:
			time.sleep(0.1)
			frame = self.cam_left_image.copy()
			frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# 计算光流
			p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
			#print ("new point:",p1[st==1])
			#print("old point:",p0[st==1])
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

			# cv2.imshow('frame',img)
			k = cv2.waitKey(30) & 0xff
			if k == 27:
				break

			# 更新上一帧的图像和追踪点
			old_gray = frame_gray.copy()
			cv2.imshow("old gray", old_gray)
			cv2.imshow("frame", frame)
			p0 = good_new.reshape(-1,1,2)
			print("old point",p0)
			#print(str(j)+":", p0)
			
		cv2.destroyAllWindows()


	#This function is desigend for mathcing the user's face, simple way to check the 
	#identity of people who is using MIRO, after MIRO is activated, it will ask for checking
	#the face of the user.
	def detect_match_target(self):
		r_pos = []
		l_pos = []
		# while True:
			# l,r = self.get_raw_image()
			# time.sleep(2)
			# l_pos = self.get_match_template(l,True)
			# l_pos = self.get_match_template(self.camera.cam_left_image,True)
			# time.sleep(2)
			# r_pos = self.get_match_template(r,False)
			# r_pos = self.get_match_template(self.camera.cam_right_image,False)
		l_pos = self.get_match_template(True)
		print(l_pos)
		# return 	l_pos, r_pos

	def get_match_template(self,is_left):

		#choose a template from the images
		template=cv2.imread('miro_head.jpg',0)
		tw,th=template.shape[::-1]
		center_pos = []
		#using the TM_CCOEFF method to match, the result depends on the result
		#if the result volume is the largest, the area can be mathched
		time.sleep(0.1)
		image = self.cam_left_image.copy()
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#the source picture must be 8 bits or 32 bits float
		# image.convertTo(image,CV_32F)
		#unknow error, why the photo cannot be read
		rv=cv2.matchTemplate(image,template,cv2.TM_CCOEFF)
		minVal,maxVal,minLoc,maxLoc=cv2.minMaxLoc(rv)
		topLeft=maxLoc
		bottomRight=(topLeft[0]+tw,topLeft[1]+th)
		cv2.rectangle(image,topLeft,bottomRight,255,2)
		center_pos = [topLeft[0]+tw/2,topLeft[1]+th/2 ]
		if is_left:
			cv2.namedWindow("left match image")
			cv2.imshow("left match image", image)
		elif is_left == False:
			cv2.imshow("right match image", image)
		# plt.subplot(121),plt.imshow(rv,cmap='gray')
		# plt.title('Matching Result'),plt.xticks([]),plt.yticks([])
		# plt.subplot(122),plt.imshow(image,cmap='gray')
		# plt.title('Detected Point'),plt.xticks([]),plt.yticks([])
		# plt.show()
		if cv2.waitKey(1)&0xff == 27:    # esc键
			cv2.destroyAllWindows()
		return center_pos

	#change the pixel to the position overhead.
	def pixel_to_point(self, pixel_loc, range,cam_index):
		if pixel_loc != None:
			view_line = self.cam_model.p2v(pixel_loc) #Transform to view line in CAM
			point_head = self.cam_model.v2oh(cam_index, view_line, range)
			return point_head
		else:
			return None
