#!/usr/bin/python3
# -*- coding:utf-8 -*-
#
#   #############################################################################
#   # This is the core version of my previous dissertation project.             #
#   # This project is designed for off-board deployment or the simulator.       #
#   # If you want to reproduce my project result, you can clone this            #
#   # core branch, this can be easier and faster for you to implement.          #
#   # All codes are deployed off-board rather than on-board, which means all    #
#   # codes will be deployed on a local computer rather than on a miro robot.   #
#   # Functionalities include object detection and object tracking, then to     #
#   # the control module for following a target and also intercepting a target  #
#   # can be implemented.                                                       #
#   # Some codes are inspired from source codes in the MiRo project. You can    #
#   # you refer to Consequential Robotics http://consequentialrobotics.com      #
#   #############################################################################


'''
    Here is the main control module for this system. 
    All commands will be processed in this node, we have
    several nodes in this system, such as tracker node,
    detector node, neural network node and safety control node.
    Navigation node can be considered at the end. 
'''

import os
import sys
import rospy
import sensor_msgs
import std_msgs
import geometry_msgs
from miro_cv.nodes import pars # 注意这个pars的包内包含的所有的机器人的传感器相关的参数
import multiprocessing as mp
import time

# 这里导入的包，是这个client的所有必要的参数和功能包。
# Here I import all basic nodes and their necessary functions to
# support the control module.
import miro2 as miro
from cv_bridge import CvBridge # 使用CvBridge进行opencv和ros的图像信息进行转换。
from miro_cv.nodes.node_detector import*
from miro_cv.nodes.kc_initiate import *
from miro_cv.nodes.node_path_planning import *
import miro_cv.utils as utils
from miro_cv.nodes.node_direction_keeper import *
from miro_cv.nodes.node_tracker import *
from miro_cv.nodes.node_CNN_detector import *
from miro_cv.nodes.node_multitracker import *
from miro_cv.nodes.safety_control import *
import matplotlib.pyplot as plt

class MainNode:

    def __init__(self, name, cascade_path):
        # initial setup of the robot name, you need to source the mdk/setup.bash before runing
        topic_root = "/" + name + "/"
        self.is_activated = True # The default safety range control is opened, so the default value is True.
        #Initinalize some essential instances.
        self.kc = Kc_init(topic_root)
        self.detector = NormalDetector(topic_root, cascade_path)
        self.path = PathPlanner(topic_root)
        self.tracker = Tracker(topic_root)
        self.multiTracker = MultiTracker(topic_root)
        self.direction_keeper = DirectionKeeper(topic_root)
        self.NN_detector = NN_detector(topic_root)
        self.safety_controller = SafetyController(topic_root)

    def init_balls_detection_l(self):
        bbox, output = self.detector.detect_balls_l("#0000FF")
        return bbox, output
        
    def init_balls_detection_r(self):
        bbox, output = self.detector.detect_balls_r("#0000FF")
        return bbox, output

    # Pick a color you want to detect, sometimes exterior light source will affect the detection
    def init_ball_detection_l(self):#0000FF #DE3163 #DC143C #E32636 #FF4D00 
        bbox, output = self.detector.detect_ball_l("#0000FF")
        return bbox, output
        
    def init_ball_detection_r(self):
        bbox, output = self.detector.detect_ball_r("#0000FF")
        return bbox, output

    def init_tracker_s(self,bbox,image,is_left):
        angular_vel , angle_lst  = self.tracker.tracking_motion_s(bbox, image, is_left)
        return angular_vel , angle_lst

    def init_tracker(self,l_bbox, r_bbox,l_output, r_output):
        l_angle_lst , r_angle_lst, r_angular_vel, l_angular_vel = self.tracker.tracking_motion(l_bbox, r_bbox,l_output, r_output)
        return l_angle_lst , r_angle_lst, r_angular_vel, l_angular_vel

    def init_multi_tracker(self,bbox_lst,cam_index):
        self.multiTracker.multitracking_motion(bbox_lst,cam_index)

    def init_multi_tracker_s(self,cam_index):
        self.multiTracker.multitracking_motion_s(cam_index)

    def init_miro_detection_l(self):
        bbox_lst_l , outputl = self.detector.detect_MIRO_l()
        return bbox_lst_l , outputl

    def init_miro_detection_r(self):
        bbox_lst_r , outputr = self.detector.detect_MIRO_r()
        return bbox_lst_r , outputr

    def init_pedestrain_detection(self):
        self.detector.detect_pedestrian()
    
    def init_movement_detection(self):
        self.detector.detect_movement()

    def init_kinematic(self):
        self.kc.init_kc()

    def init_path_planning(self):
        self.path.loop()

    #This module will not be used in the system, I want to design a combined face detection and identification algorithm.
    #TODO create a function for face detection and face identification
    def init_match_target(self):
        # self.detector.detect_match_target()
        self.detector.get_match_template(True)

    #Here is the init of a safety controller with the range 0.15 meter.
    def init_safety_controller(self):
        self.is_activated = self.safety_controller.sonar_control(0.15)
        return self.is_activated


    # The following functions are some neural network models can be used and tested in this system.
    # Some functions are still in the testing stage, I will use a stable model in the following main function later.
    # Every camera will initiate the neural network model once.
    def init_NN_detection_caffe_l(self):
        bbox, output = self.NN_detector.detect_targets_caffe('../lib/mobilenet_ssd_caffe/MobileNetSSD_deploy.prototxt.txt', '../lib/mobilenet_ssd_caffe/MobileNetSSD_deploy.caffemodel',True)
        return bbox, output
    def init_NN_detection_caffe_r(self):
        bbox, output = self.NN_detector.detect_targets_caffe('../lib/mobilenet_ssd_caffe/MobileNetSSD_deploy.prototxt.txt', '../lib/mobilenet_ssd_caffe/MobileNetSSD_deploy.caffemodel',False)
        return bbox, output	
    def init_NN_detection_squeeze_l(self):
        bbox, output = self.NN_detector.detect_targets_caffe('../lib/squeezenet_v1.1/deploy.prototxt.txt', '../lib/squeezenet_v1.1/squeezenet_v1.1.caffemodel',True)
        return bbox, output
    def init_NN_detection_squeeze_r(self):
        bbox, output = self.NN_detector.detect_targets_caffe('../lib/squeezenet_v1.1/deploy.prototxt.txt', '../lib/squeezenet_v1.1/squeezenet_v1.1.caffemodel',False)
        return bbox, output	
    def init_NN_detection_YOLO_l(self):
        bbox, output = self.NN_detector.detect_targets_darknet('../lib/YOLO_v3/yolov3.cfg', '../lib/YOLO_v3/yolov3.weights',True)
        return bbox, output
    def init_NN_detection_YOLO_r(self):
        bbox, output = self.NN_detector.detect_targets_darknet('../lib/YOLO_v3/yolov3.cfg', '../lib/YOLO_v3/yolov3.weights',False)
        return bbox, output	
    def init_NN_detection_tensorflow_mobile_ssd_v1_l(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/ssd_mobilenet_v1/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v1/graph.pbtxt',True)
        return bbox, output	
    def init_NN_detection_tensorflow_mobile_ssd_v1_r(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/ssd_mobilenet_v1/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v1/graph.pbtxt',False)
        return bbox, output
    def init_NN_detection_tensorflow_mobile_ssd_v2_l(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/ssd_mobilenet_v2/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v2/graph.pbtxt.txt',True)
        return bbox, output
    def init_NN_detection_tensorflow_mobile_ssd_v2_r(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/ssd_mobilenet_v2/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v2/graph.pbtxt.txt',False)
        return bbox, output
    def init_NN_detection_tensorflow_mask_rcnn_l(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/mask_rcnn_inception_v2/frozen_inference_graph.pb', '../lib/mask_rcnn_inception_v2/graph.pbtxt', True)		
        return bbox, output	
    def init_NN_detection_tensorflow_mask_rcnn_r(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/mask_rcnn_inception_v2/frozen_inference_graph.pb', '../lib/mask_rcnn_inception_v2/graph.pbtxt', False)	
        return bbox, output	
    def init_NN_detection_tensorflow_faster_rcnn_resnet_l(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/faster_rcnn_resnet101/frozen_inference_graph.pb', '../lib/faster_rcnn_resnet101/graph.pbtxt',True)
        return bbox, output	
    def init_NN_detection_tensorflow_faster_rcnn_resnet_r(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/faster_rcnn_resnet101/frozen_inference_graph.pb', '../lib/faster_rcnn_resnet101/graph.pbtxt',False)
        return bbox, output
    def init_NN_detection_tensorflow_faster_rcnn_resnet50_l(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/faster_rcnn_resnet50/frozen_inference_graph.pb', '../lib/faster_rcnn_resnet50/graph.pbtxt',True)
        return bbox, output	
    def init_NN_detection_tensorflow_faster_rcnn_resnet50_r(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow('../lib/faster_rcnn_resnet50/frozen_inference_graph.pb', '../lib/faster_rcnn_resnet50/graph.pbtxt',False)
        return bbox, output
    def init_NN_detection_tensorflow_mobile_ssd_v1_l_s(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow_s('../lib/ssd_mobilenet_v1/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v1/graph.pbtxt',True)
        return bbox, output
    def init_NN_detection_tensorflow_mobile_ssd_v1_r_s(self):
        bbox, output = self.NN_detector.detect_targets_tensorflow_s('../lib/ssd_mobilenet_v1/frozen_inference_graph.pb', '../lib/ssd_mobilenet_v1/graph.pbtxt',False)
        return bbox, output

    # A function for testing.
    def test(self):
        self.path.init_pos()


# The main function of this control system
if __name__ == "__main__":
    # init ROS
    pars = pars.CorePars() # init the miro_robot_name using default value
    topic_base_name = pars.ros.robot_name
    rospy.init_node(topic_base_name + "_client_demo") # log_level=self.pars.ros.log_level
    cascade_pth = rospy.get_param("/cascade_path")
    print ("<<<<Starting the " + topic_base_name + "_client_demo !!!>>>>")
    main = MainNode(topic_base_name, cascade_pth)
    main.init_kinematic()# Initiate the kinematic status of MiRo

    # ######################################################################## #
    # Module Test codes, after all modules have been tested, we can integrate  #
    # them into the main client module.										   #
    #########################################################################  #

    # main.init_movement_detection()
    # main.init_pedestrain_detection()
    # main.init_NN_detection_caffe_l()
    # main.init_NN_detection_tensorflow_mask_rcnn_l()
    # main.init_NN_detection_tensorflow_mobile_ssd_v2_l()
    # main.init_NN_detection_tensorflow_mobile_ssd_v1_l()
    # main.init_match_target()
    # main.test()
    # main.init_path_planning()
    # main.multiprocess_ball()
    # main.init_miro_detection_l()
    # main.init_miro_detection_r()
    # main.init_ball_detection_l()
    # main.init_ball_detection_r()

    # MiRo detection and ball detection modules using normal opencv APIs
    # Single target to be tracked. If you only want to track one target,
    # you can use this code part, here I only use the ball detection module.
    # As in the simulator, ball detection owns the best effect, we can easily
    # find how the whole system works. I leave this code section here for testing.
    # Here we need a loop to intiate this dynamic system.

    # l_bbox = ()
    # r_bbox = ()
    # ball_l_bbox_lst= []
    # ball_r_bbox_lst= []
    # angular_vel = []
    # angle_lst = []
    data_resolver = utils.data_resolver.DataResolver()

    # Set a loop for the system for testing, may use while main.is_activated ==True:
    for i in range(10):
        # This is the safety controller, every time it will check the sonar sensor to judge if the 
        # the distance between the robot and the target or other objects is less than 0.15,
        main.is_activated = main.init_safety_controller()
        # MiRo detection block, ouput the detected bbox and the output image
        miro_l_bbox_lst, l_output = main.init_miro_detection_l()
        miro_r_bbox_lst, r_output = main.init_miro_detection_r()

        # Ball detection block
        # ball_l_bbox_lst, l_output = main.init_ball_detection_l()
        # ball_r_bbox_lst, r_output= main.init_ball_detection_r()

        # clean up null tuples in the list
        # l_bbox_lst = [x for x in ball_l_bbox_lst if x]
        # r_bbox_lst = [x for x in ball_r_bbox_lst if x]

        # clean up null tuples in the list 清除空元组在列表中
        l_bbox_lst = [x for x in miro_l_bbox_lst if x]  # detected bbox on the left camera
        r_bbox_lst = [x for x in miro_r_bbox_lst if x]  # detected bbox on the right camera

        if len(r_bbox_lst) != 0 and len(l_bbox_lst) != 0:
            r_bbox = data_resolver.box_resolver(r_bbox_lst)
            l_bbox = data_resolver.box_resolver(l_bbox_lst)
            l_angle_lst , r_angle_lst, r_angular_vel, l_angular_vel = main.init_tracker(l_bbox, r_bbox,l_output, r_output)

        elif len(r_bbox_lst) == 0 and len(l_bbox_lst) != 0:
            print("No targets at the right side, activate tracking left side.")
            l_bbox = data_resolver.box_resolver(l_bbox_lst)
            angular_vel , angle_lst = main.init_tracker_s(l_bbox, l_output, 0)

        elif len(r_bbox_lst) !=0 and len(l_bbox_lst) == 0:
            print("No targets at the left side, activate tracking right side.")
            r_bbox = data_resolver.box_resolver(r_bbox_lst)
            angular_vel , angle_lst = main.init_tracker_s(r_bbox, r_output, 1)

        elif len(r_bbox_lst) == 0 and len(l_bbox_lst) == 0:
            print("No targets at two sides.")

        if main.is_activated is not True:
            print ("Too close!!!")
            break

    # Painting the diagram for viewing the process of angle and angular vel
    # fig = plt.figure()
    # ax1 = fig.add_subplot(1,2,1)
    # ax1.plot(angular_vel,angle_lst, ':')
    # plt.xlabel('Angular Velocity')
    # plt.ylabel('Angle Correction')
    # plt.show()




