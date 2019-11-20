# -*- coding: utf-8 -*-
#!/usr/bin/python
#This is the node for protecting the robot and the overall system of MIRO.
#we have several modules for contorling the safety.
#1. we have a safety range through checking the distance between the front object and 
#the sonar sensor. when calculating the distance, we have to use the location of the 
# LOC_SONAR_FOVEA_HEAD
import miro2 as miro
import rospy

class SafetyController:

	def __init__(self,name):

		#Subscribe to sensors
		self.sensors_sub = rospy.Subscriber(name + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.sonar_range = None
	
	def sensors_callback(self, message):
		self.sonar_range = message.sonar.range

	def sonar_control(self,range):
		activated = True
		if self.sonar_range <= range:
			activated = False
		return activated