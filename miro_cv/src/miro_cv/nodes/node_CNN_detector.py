from imutils.video import VideoStream
from imutils.video import FPS
import imutils
from multiprocessing import Process
from multiprocessing import Queue
import multiprocessing as mp

import miro2 as miro
import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage



class NN_detector:
	bbox = ()
	def __init__(self,name):
		# rospy.init_node(os.getenv("MIRO_ROBOT_NAME") + )
		# global bbox
		self.topic_root =  name 
		#set the default size of the camera
		self.cam_model = miro.lib.CameraModel()
		#Using the default size of camera
		self.frame_w = 0
		self.frame_h = 0
		# Arrays to hold image topics
		self.cam_left_image = None
		self.cam_right_image = None
		# self.bbox = () #bounding box is a tuple format
		# Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()
		self.cam_left_sub = rospy.Subscriber(
			self.topic_root + "sensors/caml/compressed", CompressedImage,  self.cam_left_callback,queue_size=1,buff_size=52428800)
		self.cam_right_sub = rospy.Subscriber(
			self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback,queue_size=1,buff_size=52428800)


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

	def classify_frame_caffe(self,net, inputQueue, outputQueue):
		# keep looping
		while True:
			# check to see if there is a frame in our input queue
			if not inputQueue.empty():
				# grab the frame from the input queue, resize it, and
				# construct a blob from it
				frame = inputQueue.get()
				frame = cv2.resize(frame, (300, 300))
				blob = cv2.dnn.blobFromImage(frame, 0.007843,
					(300, 300), 127.5)

				# set the blob as input to our deep learning object
				# detector and obtain the detections
				net.setInput(blob)
				detections = net.forward()

				# write the detections to the output queue
				outputQueue.put(detections)

	def classify_frame_darknet(self,net, inputQueue, outputQueue):
		# keep looping
		while True:
			# check to see if there is a frame in our input queue
			if not inputQueue.empty():
				# grab the frame from the input queue, resize it, and
				# construct a blob from it
				frame = inputQueue.get()
				frame = cv2.resize(frame, (416, 416))
				blob = cv2.dnn.blobFromImage(frame,1/255, (416, 416),[0,0,0],1, False)

				# set the blob as input to our deep learning object
				# detector and obtain the detections
				net.setInput(blob)
				detections = net.forward()

				# write the detections to the output queue
				outputQueue.put(detections)

	def classify_frame_tensorflow(self,net, inputQueue, outputQueue):
		# keep looping
		while True:
			# check to see if there is a frame in our input queue
			if not inputQueue.empty():
				# grab the frame from the input queue, resize it, and
				# construct a blob from it
				frame = inputQueue.get()
				frame = cv2.resize(frame, (300, 300))
				blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True, crop=False)

				# set the blob as input to our deep learning object
				# detector and obtain the detections
				net.setInput(blob)
				detections = net.forward()

				# write the detections to the output queue
				outputQueue.put(detections)

	def detect_targets_caffe(self,prototxt,model,is_left):
		bbox = ()
		# initialize the list of class labels MobileNet SSD was trained to
		# detect, then generate a set of bounding box colors for each class
		CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
			"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
			"dog", "horse", "motorbike","person","pottedplant", "sheep",
			"sofa", "train", "tvmonitor"]
		COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
		# load our serialized model from disk
		print("[INFO] loading model...")
		net = cv2.dnn.readNetFromCaffe(prototxt,model)

		# initialize the input queue (frames), output queue (detections),
		# and the list of actual detections returned by the child process
		inputQueue = Queue(maxsize=1)
		outputQueue = Queue(maxsize=1)
		detections = None

		# construct a child process *indepedent* from our main process of
		# execution
		print("[INFO] starting process...")
		p = Process(target=self.classify_frame_caffe, args=(net, inputQueue,
			outputQueue))
		p.daemon = True
		p.start()

		# initialize the video stream, allow the cammera sensor to warmup,
		# and initialize the FPS counter
		print("[INFO] starting video stream...")
		fps = FPS().start()
		while True:
			if is_left :
				time.sleep(0.1)
				outputl = self.cam_left_image.copy()
				image = imutils.resize(outputl, width=400)
				(fH, fW) = image.shape[:2]
			else:
				time.sleep(0.1)
				outputr = self.cam_right_image.copy()
				image = imutils.resize(outputr, width=400)
				(fH, fW) = image.shape[:2]

			# if the input queue *is* empty, give the current image to
			# classify
			if inputQueue.empty():
				inputQueue.put(image)

			# if the output queue *is not* empty, grab the detections
			if not outputQueue.empty():
				detections = outputQueue.get()

			# check to see if our detectios are not None (and if so, we'll
			# draw the detections on the image)
			if detections is not None:
				# loop over the detections
				for i in np.arange(0, detections.shape[2]):
					# extract the confidence (i.e., probability) associated
					# with the prediction
					confidence = detections[0, 0, i, 2]

					# filter out weak detections by ensuring the `confidence`
					# is greater than the minimum confidence
					if confidence < 0.7:
						continue

					# otherwise, extract the index of the class label from
					# the `detections`, then compute the (x, y)-coordinates
					# of the bounding box for the object
					idx = int(detections[0, 0, i, 1])
					dims = np.array([fW, fH, fW, fH])
					box = detections[0, 0, i, 3:7] * dims
					(startX, startY, endX, endY) = box.astype("int")
					#The 15th label is person
					if idx == 15:
						# draw the prediction on the image
						label = "{}: {:.2f}%".format(CLASSES[idx],
							confidence * 100)
						cv2.rectangle(image, (startX, startY), (endX, endY),
							COLORS[idx], 2)
						y = startY - 15 if startY - 15 > 15 else startY + 15
						cv2.putText(image, label, (startX, y),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
						endX = (endX*640)/400
						endY = (endY*360)/225
						startX = (startX*640)/400
						startY = (startY*360)/225
						width = (endX - startX) 
						height = (endY - startY)
						bbox = (startX, startY, width, height)

			# show the output frame
			cv2.imshow("image", image)
			key = cv2.waitKey(1) & 0xFF
			# if the `q` key was pressed, break from the loop
			if key == ord("q"):
				break
			# update the FPS counter
			fps.update()
		# stop the timer and display FPS information
		fps.stop()
		print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
		print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

		# do a bit of cleanup
		cv2.destroyAllWindows()
		if is_left:
			return bbox ,outputl
		else:
			return bbox ,outputr

	#Here i use the mobilenet v2_ssd to implement the obejct detection, it has been
	# tested that the speed and accuracy of this model is better than last generation
	# we can use the result to see whether this conclusion is correct.
	def detect_targets_tensorflow(self,pb,pbtxt,is_left):
		bbox =()
		# img_raw_l = None
		# img_raw_r = None
		CLASSES = self.read_file("../nodes/mscoco_labels.names")
		# CLASSES = self.read_file("kitti_labels.names")
		COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
		# load our serialized model from disk
		print("[INFO] loading model...")
		cvNet = cv2.dnn.readNetFromTensorflow(pb,pbtxt)
		
		# initialize the input queue (frames), output queue (detections),
		# and the list of actual detections returned by the child process
		inputQueue = Queue(maxsize=1)
		outputQueue = Queue(maxsize=1)
		detections = None

		# construct a child process *indepedent* from our main process of
		# execution
		print("[INFO] starting process...")
		p = Process(target=self.classify_frame_tensorflow, args=(cvNet, inputQueue,
			outputQueue))
		p.daemon = True
		p.start()

		# initialize the video stream, allow the cammera sensor to warmup,
		# and initialize the FPS counter
		print("[INFO] starting video stream...")
		fps = FPS().start()
		while True:
			if is_left :
				time.sleep(0.1)
				outputl = self.cam_left_image.copy()
				image = imutils.resize(outputl, width=400)
				(fH, fW) = image.shape[:2]
			else:
				time.sleep(0.1)
				outputr = self.cam_right_image.copy()
				image = imutils.resize(outputr, width=400)
				(fH, fW) = image.shape[:2]

			# if the input queue *is* empty, give the current image to
			# classify
			if inputQueue.empty():
				inputQueue.put(image)

			# if the output queue *is not* empty, grab the detections
			if not outputQueue.empty():
				detections = outputQueue.get()

			# check to see if our detectios are not None (and if so, we'll
			# draw the detections on the image)
			# [batchId, classId, confidence, left, top, right, bottom]
			if detections is not None:
				# loop over the detections
				for i in np.arange(0, detections.shape[2]):
					# extract the confidence (i.e., probability) associated
					# with the prediction
					confidence = detections[0, 0, i, 2]
					# filter out weak detections by ensuring the `confidence`
					# is greater than the minimum confidence
					if confidence < 0.7:
						continue

					# otherwise, extract the index of the class label from
					# the `detections`, then compute the (x, y)-coordinates
					# of the bounding box for the object
					idx = int(detections[0, 0, i, 1])
					dims = np.array([fW, fH, fW, fH])
					box = detections[0, 0, i, 3:7] * dims
					(startX, startY, endX, endY) = box.astype("int")

					#62 is chair, 73 is laptop, 1 is person
					if idx == 1:
						# draw the prediction on the image
						label = "{}: {:.2f}%".format(CLASSES[idx],
							confidence * 100)
						cv2.rectangle(image, (startX, startY), (endX, endY),
							COLORS[idx], 2)
						y = startY - 15 if startY - 15 > 15 else startY + 15
						cv2.putText(image, label, (startX, y),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

						endX = (endX*640)/400
						endY = (endY*360)/225
						startX = (startX*640)/400
						startY = (startY*360)/225
						width = (endX - startX) 
						height = (endY - startY)
						bbox = (startX, startY, width,height)


			cv2.imshow('image', image)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
			# update the FPS counter
			fps.update()
		# stop the timer and display FPS information
		fps.stop()
		print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
		print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
		cv2.destroyAllWindows()
		if is_left:
			return bbox ,outputl
		else:
			return bbox ,outputr

	#This tensorflow module is designed for comparing the FPS between the above version
	# with multiprocessing. 	
	def detect_targets_tensorflow_s(self,pb,pbtxt,is_left):
		bbox = ()
		outputl = None
		outputr = None
		CLASSES = self.read_file("../nodes/mscoco_labels.names")
		COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
		cvNet = cv2.dnn.readNetFromTensorflow(pb,pbtxt)
		# initialize the video stream, allow the cammera sensor to warmup,
		# and initialize the FPS counter
		print("[INFO] starting video stream...")
		fps = FPS().start()
		while True:
			if is_left :
				time.sleep(0.1)
				outputl = self.cam_left_image.copy()
				img = imutils.resize(outputl, width=400)
				# (fH, fW) = img.shape[:2]
			else:
				time.sleep(0.1)
				outputr = self.cam_right_image.copy()
				img = imutils.resize(outputr, width=400)
				# (fH, fW) = img.shape[:2]
			rows = img.shape[0]
			cols = img.shape[1]
			
			cvNet.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
			cvOut = cvNet.forward()
			if cvOut is not None:
				for detection in cvOut[0,0,:,:]:
					score = float(detection[2])
					idx = int(detection[1])
					if score > 0.5:
						left = detection[3] * cols
						top = detection[4] * rows
						right = detection[5] * cols
						bottom = detection[6] * rows
						box = np.array([left,top,right,bottom])
						(startX, startY, endX, endY) = box.astype("int")
						#testing the pedestrian module
						# if idx == 1:
						# draw the prediction on the image
						label = "{}: {:.2f}%".format(CLASSES[idx],
							score * 100)
						endX = (endX*640)/400
						endY = (endY*360)/225
						startX = (startX*640)/400
						startY = (startY*360)/225
						width = (endX - startX) 
						height = (endY - startY)
						bbox = (startX, startY, width,height)
						cv2.rectangle(img, (startX, startY), (endX, endY),
							COLORS[idx], 2)
						y = startY - 15 if startY - 15 > 15 else startY + 15
						cv2.putText(img, label, (startX, y),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
						# bbox = tuple(bbox) 
						# img = cv2.resize(img,(640,360))
						# print "changed bbox--------------->",bbox
						# if is_left:
						# 	cv2.rectangle(outputl, (startX, startY), (endX, endY),
						# 		COLORS[idx], 2)
						# 	y = startY - 15 if startY - 15 > 15 else startY + 15
						# 	cv2.putText(outputl, label, (startX, y),
						# 		cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
						# 	cv2.imshow('outputl', outputl)
						# else:
						# 	cv2.rectangle(outputr, (startX, startY), (endX, endY),
						# 		COLORS[idx], 2)
						# 	y = startY - 15 if startY - 15 > 15 else startY + 15
						# 	cv2.putText(outputr, label, (startX, y),
						# 		cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
						# 	cv2.imshow('outputr', outputr)
			cv2.imshow('img', img)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
			# update the FPS counter
			fps.update()
		# stop the timer and display FPS information
		fps.stop()
		print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
		print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
		cv2.destroyAllWindows()
		if is_left:
			return bbox ,outputl
		else:
			return bbox ,outputr

	#This function can read the names file which contains the labels of different sources.(Kitti, COCO)
	def read_file(self,filename):
		classesFile = filename
		classes = None
		with open(classesFile, 'rt') as f:
			classes = f.read().rstrip('\n').split('\n')
		return	classes


	#Here i use the mobilenet v2_ssd to implement the obejct detection, it has been
	# tested that the speed and accuracy of this model is better than last generation
	# we can use the result to see whether this conclusion is correct.
	# def detect_targets_darknet(self,cfg,weights,is_left):
	# 	bbox =()
	# 	scores = []
	# 	outputs = None
	# 	CLASSES = self.read_file("/home/lizheng/Documents/MiRo_3.0/MIRO_demos_sim/Demo/nodes/mscoco_labels.names")
	# 	# CLASSES = self.read_file("kitti_labels.names")
	# 	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
	# 	# load our serialized model from disk
	# 	print("[INFO] loading model...")
	# 	cvNet = cv2.dnn.readNetFromDarknet(cfg,weights)
	# 	cvNet.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
	# 	cvNet.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
	# 	# initialize the input queue (frames), output queue (detections),
	# 	# and the list of actual detections returned by the child process
	# 	inputQueue = Queue(maxsize=1)
	# 	outputQueue = Queue(maxsize=1)
	# 	detections = None

	# 	# construct a child process *indepedent* from our main process of
	# 	# execution
	# 	print("[INFO] starting process...")
	# 	p = Process(target=self.classify_frame_darknet, args=(cvNet, inputQueue,
	# 		outputQueue))
	# 	p.daemon = True
	# 	p.start()

	# 	# initialize the video stream, allow the cammera sensor to warmup,
	# 	# and initialize the FPS counter
	# 	print("[INFO] starting video stream...")
	# 	fps = FPS().start()
	# 	while True:
	# 		if is_left :
	# 			time.sleep(0.1)
	# 			outputl = self.cam_left_image.copy()
	# 			image = imutils.resize(outputl, width=500)
	# 			(fH, fW) = image.shape[:2]
	# 		else:
	# 			time.sleep(0.1)
	# 			outputr = self.cam_right_image.copy()
	# 			image = imutils.resize(outputr, width=500)
	# 			(fH, fW) = image.shape[:2]

	# 		# if the input queue *is* empty, give the current image to
	# 		# classify
	# 		if inputQueue.empty():
	# 			inputQueue.put(image)

	# 		# if the output queue *is not* empty, grab the detections
	# 		if not outputQueue.empty():
	# 			outputs = outputQueue.get()
			
	# 		# print outputs.s
	# 		# check to see if our detectios are not None (and if so, we'll
	# 		# draw the outputs on the image)
	# 		# [batchId, classId, confidence, left, top, right, bottom]
	# 		img_height, img_width, _ = image.shape
	# 		if outputs is not None:
	# 			for detection in outputs[0,0,:,:]:
	# 				print detection
	# 				score = float(detection[2])
	# 				idx = int(detection[1])
	# 				# for detection in output:
	# 				# 	scores = detection[5:]
						
	# 				# idx = np.argmax(scores)
	# 				confidence = scores[idx]
	# 				if confidence > 0.2:
	# 					center_x = int(detection[0] * img_width)
	# 					center_y = int(detection[1] * img_height)
	# 					width = int(detection[2] * img_width)
	# 					height = int(detection[3] * img_height)
	# 					left = int(center_x - width / 2)
	# 					top = int(center_y - height / 2)
	# 					box = np.array([left,top, width, height])
	# 					(startX, startY, endX, endY) = box.astype("int")

	# 					#62 is chair, 73 is laptop, 1 is person
	# 					# if idx == 62:
	# 					# draw the prediction on the image
	# 					label = "{}: {:.2f}%".format(CLASSES[idx],
	# 						confidence * 100)
	# 					cv2.rectangle(image, (startX, startY), (endX, endY),
	# 						COLORS[idx], 2)
	# 					y = startY - 15 if startY - 15 > 15 else startY + 15
	# 					cv2.putText(image, label, (startX, y),
	# 						cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

	# 					endX = (endX*640)/416
	# 					endY = (endY*360)/209
	# 					startX = (startX*640)/416
	# 					startY = (startY*360)/209
	# 					width = (endX - startX) 
	# 					height = (endY - startY)
	# 					bbox = (startX, startY, width,height)
	# 					# detections = cv2.dnn.NMSBoxes()
	# 		cv2.imshow('image', image)
	# 		key = cv2.waitKey(1) & 0xFF
	# 		if key == ord("q"):
	# 			break
	# 		# update the FPS counter
	# 		fps.update()
	# 	# stop the timer and display FPS information
	# 	fps.stop()
	# 	print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
	# 	print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
	# 	cv2.destroyAllWindows()
	# 	if is_left:
	# 		return bbox ,outputl
	# 	else:
	# 		return bbox ,outputr