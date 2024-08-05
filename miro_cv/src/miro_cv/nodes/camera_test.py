import miro2 as miro
import rospy
# Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage

from camera_callback import *
import time
import os



#get raw image 
def get_raw_image():
    topic_root = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"
    camera = Camera(topic_root)
    while True:
        
        time.sleep(2)
        print(camera.cam_left_image)
        # l_image = 
        cv2.imshow("left", camera.cam_left_image)
        time.sleep(1)
        # r_image = 
        cv2.imshow("right", camera.cam_right_image)



get_raw_image()