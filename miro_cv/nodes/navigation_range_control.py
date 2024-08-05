#This is a module for controlling the navigation range, it means there is a maxinum
# range for a robot. In another word, the robot cannot leave the area which it is 
#set by human operator. In the simulator or the real world, my idea is that the floor
#can be circled with some dark color lines. So, we can use the cliff sensor to keep the
# robot not leaving the circle range, I call it security area.

import miro2 as miro
import rospy

class NavController:

	def __init__(self,name):

		#Subscribe to sensors
		self.sensors_sub = rospy.Subscriber(name + "sensors/package", miro.msg.sensors_package, self.sensors_callback)
		self.cliff_data = None

	def sensors_callback(self, message):
		self.cliff_data = message.cliff.data

	def cliff_control(self):
		time = 0
		is_danger = False
		if self.cliff_data <=0.1:
			is_danger = True
		return is_danger