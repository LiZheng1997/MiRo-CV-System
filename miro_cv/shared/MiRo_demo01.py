# -*- coding: utf-8 -*-

import miro2 as miro
#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import rospy

#尝试直接使用MIRO中的interface接口去调用 已经存在的找球的函数

class controller:

	def __init__(self):
		# Arrays to hold image topics
		self.cam_left_image =None
		self.cam_right_image = None

	def loop(self):

		platformInter = miro.interface.interface.PlatformInterface("miro")

		colour_str = "#0000CD"
		#read images from the camera frame by frame
		max_circle_norm = platformInter.find_ball(colour_str,0,1)
		print(max_circle_norm)
		# for i in max_circle_norm[:]:
		#     print(i)
		time.sleep(1)
		self.cam_left_image = platformInter.cam_left_image
		cv2.imshow("output",self.cam_left_image)

		cv2.waitKey()
		cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.init_node("demo01", anonymous=True)
	contr = controller()
	contr.loop()