#!/usr/bin/python
#
#	{{##file_header##}}
#
#	This is a MIRO ROS client for herding Scottish children.
#
#	This example implements a Braitenberg vehicle type 2a.
#	https://en.wikipedia.org/wiki/Braitenberg_vehicle

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage

import math
import numpy as np
import time
import sys
import os

from pynput import keyboard
import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class controller:

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# store
		self.package = msg


	def loop(self):

		# output
		msg_cmd_vel = TwistStamped()

		# desired wheel speed (m/sec)
		wheel_speed = [0.0, 0.0]
		v = 1.0

		# loop
		while self.active and not rospy.core.is_shutdown():

			if self.cmd == 'LEFT':
				wheel_speed[0] = 0.0
				wheel_speed[1] = v
			elif self.cmd == 'RIGHT':
				wheel_speed[0] = v
				wheel_speed[1] = 0.0
			elif self.cmd == 'UP':
				wheel_speed[0] = v
				wheel_speed[1] = v
				print 'Moving miro up' 
			else:
				wheel_speed[0] = 0.0
				wheel_speed[1] = 0.0
			
		


			# convert wheel speed to command velocity (m/sec, Rad/sec)
			(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)
			
			print dr

			# update message to publish to control/cmd_vel
			msg_cmd_vel.twist.linear.x = dr
			msg_cmd_vel.twist.angular.z = dtheta

			# publish message to topic
			self.pub_cmd_vel.publish(msg_cmd_vel)


			# yield
			time.sleep(0.01)
			self.t_now = self.t_now + 0.01

	def on_press(self,key):
		try:
			c = key.char
			#print('alphanumeric key {0} pressed'.format(c))

			if c == 'i':
				self.cmd = 'UP'
			elif c == 'j':
				self.cmd = 'LEFT'
			elif c == 'l':
				self.cmd = 'RIGHT'
			else:
				self.cmd = None
		except AttributeError:
			print('special key {0} pressed'.format(
				key))

	def on_release(self,key):
		#print('{0} released'.format(key))
		self.cmd = None
		if key == keyboard.Key.esc:
			# Stop listener
			return False

	def __init__(self, args):


		rospy.init_node("client_shepherd", anonymous=True)

		# state
		self.t_now = 0.0
		self.wheel_speed = [0.0, 0.0]

		# inputs
		self.active = False
		self.cmd = None

		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "pass":
				pass
			else:
				error("argument not recognised \"" + arg + "\"")

		# Listening to the keyboard
		'''
		with keyboard.Listener(
				on_press=self.on_press,
				on_release=self.on_release) as listener:
			listener.join()
		'''
		

		# robot name
		# topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = "/miro/control/cmd_vel"
		print ("publish", topic)
		self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# subscribe
		topic = "/miro/sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)


		# wait for connect
		print "wait for connect..."
		time.sleep(1)

		# set to active
		self.active = True

		listener = keyboard.Listener(
		    on_press=self.on_press,
		    on_release=self.on_release)
		listener.start()

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()




