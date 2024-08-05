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


#完成移动到一个指定位置的任务。提取物体的基本的信息。

