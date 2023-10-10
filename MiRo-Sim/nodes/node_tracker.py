# -*- coding: utf-8 -*-
#########################################
### how to choose a tracker in OpenCV ###
#########################################

#
import datetime
import miro2 as miro
import numpy as np
import cv2 as cv
import sys
import rospy
import time
import node_path_planning

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from node_direction_keeper import *
from geometry_msgs.msg import Pose2D,Vector3
from safety_control import *
from utils import data_resolver

class Tracker:

    def __init__(self,name):
        self.topic_root =  name 
        self.cam_model = miro.lib.CameraModel()
        #Using the default size of camera
        self.frame_w = 0
        self.frame_h = 0
        
        self.pose = Pose2D()
        self.velocity = TwistStamped()
        # Arrays to hold image topics
        self.cam_left_image = None
        self.cam_right_image = None
        # Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()
        self.direction_keeper = DirectionKeeper(self.topic_root)
        self.data_resolver = data_resolver.DataResolver()
        self.global_planner = node_path_planning.PathPlanner(name)
        self.is_activated = True
        self.safety_controller = SafetyController(name)

        #publisher
        self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
        self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)

        #suscriber
        self.cam_left_sub = rospy.Subscriber(
            self.topic_root + "sensors/caml/compressed", CompressedImage,  self.cam_left_callback,queue_size=1,buff_size=52428800)
        self.cam_right_sub = rospy.Subscriber(
            self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback,queue_size=1,buff_size=52428800)
        self.pos_sub =  rospy.Subscriber(
            self.topic_root + "sensors/body_pose", Pose2D, self.pose_callback,queue_size=1,buff_size=52428800)
        self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.callback_sensors)

        self.current_mes = np.array((2,1),np.float32)
        self.current_pre = np.array((2,1),np.float32)
        self.kalman = cv2.KalmanFilter(4,2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 0.003
        self.kalman.measurementNoiseCov = np.array([[1,0],[0,1]], np.float32) * 1


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

    def callback_sensors(self,msg):
        self.sensors_pack = msg

    #monocular way for tracking
    def tracking_motion_s(self,bbox,image,cam_index):
        #check the opencv version
        (major_ver, minor_ver, subminor_ver) = (cv.__version__).split('.')
        # Set up tracker.
        # Instead of MIL, you can also use
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        #choose the tracker using the index in the list above
        tracker_type = tracker_types[3]
        if tracker_type == 'BOOSTING':
            tracker = cv.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv.TrackerGOTURN_create()
        if tracker_type == 'MOSSE':
            tracker = cv.TrackerMOSSE_create()
        if tracker_type == 'CSRT':
            tracker = cv.TrackerCSRT_create()
        
        bbox = list(bbox)
        bbox[0] = bbox[0] if bbox[0]> 0 else 0
        bbox[1] = bbox[1] if bbox[1]> 0 else 0
        bbox[2] = bbox[2] if bbox[2] < 640 -bbox[0] else (640 - bbox[0] -1)
        bbox[3] = (360 - bbox[1] -1) if bbox[3] > (360 -bbox[1]) else bbox[3]
        bbox = tuple(bbox)
        print("bbox:------------------------------->",bbox)
        # bbox[0] = 0 if bbox[0]< 0 else bbox[0]
        # bbox[1] = 0 if bbox[1]< 0 else bbox[1]
        # bbox[2] = (640 - bbox[0] -1) if bbox[2] > (640 -bbox[0]) else bbox[2]
        # bbox[3] = (360 - bbox[1] -1) if bbox[3] > (360 -bbox[1]) else bbox[3]
        # bbox = tuple(bbox)
        
        #get the first bbox in the first frame from the detection module
        ok = tracker.init(image, bbox)
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
        r_pos = [0.0,0.0]
        old_r_pos = None
        #get the pose from the topic, the pose topic only exists in the simulator
        pose = np.array([self.pose.x,self.pose.y,self.pose.theta])
        angular_vel = []
        angle_lst = []

        while True:
            self.is_activated = self.safety_controller.sonar_control(0.15)
            if self.is_activated == False:
                print ("Too close!!!")
                break
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
            ok, newbox = tracker.update(frame)
            # Calculate Frames per second (FPS)
            fps = cv.getTickFrequency() / (cv.getTickCount() - now)
            box = list(newbox)
            #get the center pos of the newbox
            center_pos = [box[0]+box[2]/2, box[1]+box[3]/2]
            object_x = int(center_pos[0])
            if ok:
                p1 = (int(newbox[0]), int(newbox[1]))
                p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                # cv.rectangle(frame, p1, p2, (200,0,0))
                cv.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                angle, beta, t_pos, direction, center_line = self.direction_keeper.diret_keeper_s(center_pos,cam_index,pose)
                print("actual angle---------->:", angle)
                frame = cv.line(frame, (object_x, 180), (object_x, 360), (255, 0, 0), 1)
                frame = cv.line(frame, (object_x, 180), (center_line[0], 180), (0, 0, 255), 2)
                # frame = cv.line(frame,(object_x,180),(320,180),(0,0,255),2)
                if angle != 0 :
                    self.velocity = self.direction_keeper.reguralize_angle(angle, direction, False)
                #after the angle has been corrected, the t_pos above will be None, so we should
                #tansform the target to the world coordination system. And this can be a point for
                #later heading toward and tracking the moving object. I set a accepted error range
                #at 0.8 degree for saving time, because the angle will swing between the zero degree,
                # #so, we might not always get the lowest value, like the gradient in machine learning.
                #considering the time limitation, we cannot allow the robot to regularize
                # the angle at a extremelly accurate stage,so here i just give him a large
                #angle to push the velocity.
                # old_second =  datetime.datetime.now().second
                if old_t_pos is not None:
                    # pose = np.array([self.pose.x,self.pose.y,self.pose.theta])
                    print("pose--------->",pose)
                    vel_r, cos_l,sin_l = self.global_planner.path_planning(cam_index,center_pos,old_t_pos,beta,pose,now,old_time, self.velocity.twist.angular.z)
                    print("pose-----------------after>",pose)

                    spd = np.array(self.sensors_pack.wheel_speed_opto.data)
                    print("spd--->",spd) 
                    twist = self.sensors_pack.odom.twist.twist
                    dr = twist.linear.x
                    dtheta = twist.angular.z

                    #Here is the measurement of the state space(position, velocity)
                    # mes_array = np.array([pose.x,pose.y, dr *cos_l,dr *sin_l])
                    # mes_array = np.array([pose[0],pose[1]])
                    mes_array = np.array([old_t_pos[0],old_t_pos[1]])
                    self.current_mes =  np.array([[np.float32(mes_array[0])],[np.float32(mes_array[1])]])
                    estimated = self.kalman.correct(self.current_mes)
                    print("corrected estimation from KF",estimated) 
                    self.current_pre = self.kalman.predict()
                    print("Prediciton from Kalman Filter",self.current_pre) 
                    # print ""
                    #Here is the prediction of the state space, compare the prediction and the Kalman prediction
                    r_pos[0] = r_pos[0] +  vel_r * cos_l * 2.0
                    r_pos[1] = r_pos[1] +  vel_r * sin_l * 2.0
                    pre_array = np.array([r_pos[0],r_pos[1],vel_r * cos_l,vel_r * sin_l])
                    print("Prediction from calculation", pre_array) 
                    print("pose-----------------after>",pose)

                    # current_second = datetime.datetime.now().second
                    # if abs(current_second - old_second):
                    # 	continue
            else :
                # Tracking failure
                cv.putText(frame, "Tracking failure detected", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                cv2.destroyAllWindows()
                break
            # Display tracker type on frame
            cv.putText(frame, tracker_type + " Tracker", (100,20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on frame
            cv.putText(frame, "FPS : " + str(int(fps)), (100,50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            im_h = np.size(frame, 0)
            im_w = np.size(frame, 1)
            im_centre_h = im_h / 2.0
            im_centre_w = im_w / 2.0
            #draw the calibration lines at x and y axis in the middle of the image.
            cv.line(frame, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
            cv.line(frame, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)

            # old_r_pos = r_pos
            old_t_pos = t_pos
            old_time = now

            
            #set the frames number for this tracking module to update, it means a epoch
            frame_count += 1
            if frame_count == 200:
                cv.destroyAllWindows()
                break

            if cam_index == 0:
                cv.imshow("left_tracking", frame)
            elif cam_index == 1:
                cv.imshow("right_tracking", frame)

            k = cv.waitKey(1) & 0xff
            if k == 27 : break # esc pressed
            angular_vel.append(self.velocity.twist.angular.z)
            angle_lst.append(angle)
        return angular_vel , angle_lst

    def tracking_motion(self,l_bbox,r_bbox,l_image,r_image):

        #check the opencv version
        # (major_ver, minor_ver, subminor_ver) = (cv.__version__).split('.')
        l_tracker = cv.TrackerBoosting_create()
        r_tracker = cv.TrackerBoosting_create()

        # Set up tracker.
        # Instead of MIL, you can also use
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        #choose the tracker using the index in the list above
        l_tracker_type = tracker_types[3]
        r_tracker_type = tracker_types[2]
        if l_tracker_type  == 'BOOSTING':
            l_tracker = cv.TrackerBoosting_create()
        elif r_tracker_type  == 'BOOSTING':
            r_tracker = cv.TrackerBoosting_create()
        if l_tracker_type  == 'MIL':
            l_tracker = cv.TrackerMIL_create()
        elif r_tracker_type == 'MIL':
            r_tracker = cv.TrackerMIL_create()
        if l_tracker_type  == 'KCF':
            l_tracker = cv.TrackerKCF_create()
        elif r_tracker_type == 'KCF':
            r_tracker = cv.TrackerKCF_create()
        if l_tracker_type  == 'TLD':
            l_tracker = cv.TrackerTLD_create()
        elif r_tracker_type == 'TLD':
            r_tracker = cv.TrackerTLD_create()
        if l_tracker_type == 'MEDIANFLOW':
            l_tracker = cv.TrackerMedianFlow_create()
        elif r_tracker_type == 'MEDIANFLOW':
            r_tracker = cv.TrackerMedianFlow_create()
        if l_tracker_type == 'GOTURN':
            l_tracker = cv.TrackerGOTURN_create()
        elif r_tracker_type == 'GOTURN':
            r_tracker = cv.TrackerGOTURN_create()
        if l_tracker_type == 'MOSSE':
            l_tracker = cv.TrackerMOSSE_create()
        elif r_tracker_type == 'MOSSE':
            r_tracker = cv.TrackerMOSSE_create()
        if l_tracker_type == 'CSRT':
            l_tracker = cv.TrackerCSRT_create()
        elif r_tracker_type == 'CSRT':
            r_tracker = cv.TrackerCSRT_create()

        # check the bbox's avaliability to avoid the bbox is out of the image
        l_bbox = list(l_bbox)
        l_bbox[0] = l_bbox[0] if l_bbox[0]> 0 else 0
        l_bbox[1] = l_bbox[1] if l_bbox[1]> 0 else 0
        l_bbox[2] = l_bbox[2] if l_bbox[2] < 640 -l_bbox[0] else (640 - l_bbox[0] -1)
        l_bbox[3] = (360 - l_bbox[1] -1) if l_bbox[3] > (360 -l_bbox[1]) else l_bbox[3]
        l_bbox = tuple(l_bbox)
        print("l_bbox:------------------------------->",l_bbox)

        # check the bbox's avaliability to avoid the bbox is out of the image
        r_bbox = list(r_bbox)
        r_bbox[0] = r_bbox[0] if r_bbox[0]> 0 else 0
        r_bbox[1] = r_bbox[1] if r_bbox[1]> 0 else 0
        r_bbox[2] = r_bbox[2] if r_bbox[2] < 640 -r_bbox[0] else (640 - r_bbox[0] -1)
        r_bbox[3] = (360 - r_bbox[1] -1) if r_bbox[3] > (360 -r_bbox[1]) else r_bbox[3]
        r_bbox = tuple(r_bbox)
        print("r_bbox:------------------------------->",r_bbox)

        #get the first bbox in the first frame from the detection module
        time.sleep(0.01)
        l_ok = l_tracker.init(l_image, l_bbox)
        time.sleep(0.01)
        r_ok = r_tracker.init(r_image, r_bbox)
        #use a counter to calculate the overall frames have been updated
        frame_count = 0
        l_center_pos = None
        r_center_pos = None
        object_x = None
        angle = 0
        # old = cv2.getTickCount()
        angle_last = 0
        # old_box = None
        l_old_t_pos = None
        r_old_t_pos = None
        r_t_pos  = None
        l_t_pos = None
        pose = np.array([self.pose.x,self.pose.y,self.pose.theta])
        l_old_time = 0
        r_old_time = 0
        l_angular_vel = []
        l_angle_lst = []
        r_angular_vel = []
        r_angle_lst = []
        r_angle = 0
        l_angle = 0

        while True:
            self.is_activated = self.safety_controller.sonar_control(0.15)
            if self.is_activated == False:
                print("Too close!!!") 
                break
            #get the output of the image from camera
            time.sleep(0.1)
            l_frame = self.cam_left_image.copy()
            #get the output of the image from camera
            time.sleep(0.1)
            r_frame = self.cam_right_image.copy()
            if l_frame is None and r_frame is None:
                print("no image is readed")
                break
            # Start now, because we will need the time of updating frames by different
            #trackers, so we store the time here seperately
            l_now = cv.getTickCount()
            l_ok, l_newbox = l_tracker.update(l_frame)
            l_fps = cv.getTickFrequency() / (cv.getTickCount() - l_now)
            r_now = cv.getTickCount()
            r_ok, r_newbox = r_tracker.update(r_image)
            r_fps = cv.getTickFrequency() / (cv.getTickCount() - r_now)
            print("dif_fps---------------->:",l_fps-r_fps)

            l_box = list(l_newbox)
            r_box = list(r_newbox)
            #get the center pos of the newbox
            l_center_pos = [l_box[0]+l_box[2]/2, l_box[1]+l_box[3]/2]
            l_object_x = int(l_center_pos[0])
            #get the center pos of the newbox
            r_center_pos = [r_box[0]+r_box[2]/2, r_box[1]+r_box[3]/2]
            r_object_x = int(r_center_pos[0])
            
            if l_ok and r_ok is False:
                p1 = (int(l_newbox[0]), int(l_newbox[1]))  # bbox左上角的点
                p2 = (int(l_newbox[0] + l_newbox[2]), int(l_newbox[1] + l_newbox[3]))  # bbox右下角的点
                # cv.rectangle(frame, p1, p2, (200,0,0))
                cv.rectangle(l_frame, p1, p2, (255,0,0), 2, 1)
                l_angle, l_beta, l_t_pos, direction ,l_center_line= self.direction_keeper.diret_keeper(l_center_pos,r_center_pos,pose)
                cv.line(l_frame,(l_object_x,180),(l_object_x,360),(255,0,0),1)
                l_frame = cv.line(l_frame,(l_object_x,180),(int(l_center_line[0]),180),(0,0,255),2)
                # cv.line(l_frame,(l_object_x,180),(320,180),(0,0,255),2)
                print("left_actual angle---------->:", l_angle)
                if l_angle != 0:
                    self.velocity = self.direction_keeper.reguralize_angle(l_angle, direction, True)
                    l_angular_vel.append(self.velocity.twist.angular.z)
                # The final status is the robot's velocity lies on the sight line between the 
                #robot and the target. I set a small value for the accepted error range of degrees
                # at 1 degree, in this case, the direction of the robot is almost directed forward 
                # to the target. Also, if the old target position is None, it means there is no postion
                # info at the initiate stage, so we need to skip this result.
                #considering the time limitation, we cannot allow the robot to regularize
                # the angle at a extremelly accurate stage,so here i just give him a large
                #angle to push the velocity.  
                if l_old_t_pos is not None:
                    #update the pose from the subscriber
                    # pose = np.array([self.pose.x,self.pose.y,self.pose.theta])
                    self.global_planner.path_planning(0,l_center_pos,l_old_t_pos,l_beta,pose,l_now,l_old_time,self.velocity.twist.angular.z)
            elif l_ok != True :
                # Tracking failure
                cv.putText(l_frame, "Tracking failure detected", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                pass
            # Display tracker type on l_frame
            cv.putText(l_frame, l_tracker_type + " Tracker", (100,20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on l_frame
            cv.putText(l_frame, "FPS : " + str(int(l_fps)), (100,50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            im_h = np.size(l_frame, 0)
            im_w = np.size(l_frame, 1)
            im_centre_h = im_h / 2.0
            im_centre_w = im_w / 2.0
            #draw the calibration lines at x and y axis in the middle of the image.
            cv.line(l_frame, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
            cv.line(l_frame, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)
            # and l_ok == False
            if r_ok and l_ok == False:
                p1 = (int(r_newbox[0]), int(r_newbox[1]))
                p2 = (int(r_newbox[0] + r_newbox[2]), int(r_newbox[1] + r_newbox[3]))
                # cv.rectangle(frame, p1, p2, (200,0,0))
                cv.rectangle(r_frame, p1, p2, (255,0,0), 2, 1)
                r_angle, r_beta, r_t_pos,direction,r_center_line = self.direction_keeper.diret_keeper(l_center_pos,r_center_pos,pose)
                cv.line(r_frame,(r_object_x,180),(r_object_x,360),(255,0,0),1)
                cv.line(r_frame,(r_object_x,180),(int(r_center_line[0]),180),(0,0,255),2)
                print("left_actual angle---------->:", r_angle)
                if r_angle !=0:
                    self.velocity = self.direction_keeper.reguralize_angle(r_angle,direction, True)
                    r_angular_vel.append(self.velocity.twist.angular.z)
                # The final status is the robot's velocity lies on the sight line between the 
                #robot and the target. I set a small value for the accepted error range of degrees
                # at 1 degree, in this case, the direction of the robot is almost directed forward 
                # to the target. Also, if the old target position is None, it means there is no postion
                # info at the initiate stage, so we need to skip this result.
                #considering the time limitation, we cannot allow the robot to regularize
                # the angle at a extremelly accurate stage,so here i just give him a large
                #angle to push the velocity.  
                if r_old_t_pos is not None:
                # 	#update the pose from the subscriber
                    # pose = np.array([self.pose.x,self.pose.y,self.pose.theta])
                    self.global_planner.path_planning(1,r_center_pos,r_old_t_pos,r_beta,pose,r_now,r_old_time,self.velocity.twist.angular.z)
            elif r_ok != True :
                # Tracking failure
                cv.putText(r_frame, "Tracking failure detected", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                cv2.destroyAllWindows()
                break
            # Display tracker type on r_frame
            cv.putText(r_frame, r_tracker_type + " Tracker", (100,20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on r_frame
            cv.putText(r_frame, "FPS : " + str(int(r_fps)), (100,50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            im_h = np.size(r_frame, 0)
            im_w = np.size(r_frame, 1)
            im_centre_h = im_h / 2.0
            im_centre_w = im_w / 2.0
            #draw the calibration lines at x and y axis in the middle of the image.
            cv.line(r_frame, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
            cv.line(r_frame, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)

            #set the frames number for this tracking module to update, it means a epoch is 50 frames
            frame_count += 1
            if frame_count == 200:
                cv.destroyAllWindows()
                break

            #update the position of the target.
            r_old_t_pos = r_t_pos
            l_old_t_pos = l_t_pos
            # angle_last = angle
            l_old_time = l_now
            r_old_time = r_now

            cv.imshow("left_tracking", l_frame)
            cv.imshow("right_tracking", r_frame)

            k = cv.waitKey(1) & 0xff
            if k == 27 : break # esc pressed

            
            l_angle_lst.append(l_angle)
            r_angle_lst.append(r_angle)
            
        return  l_angle_lst , r_angle_lst, r_angular_vel, l_angular_vel