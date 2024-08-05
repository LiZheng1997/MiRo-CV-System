# -*- coding: utf-8 -*-
import numpy as np
import copy
import miro2 as miro
import miro_cv.nodes.node_change_kc as node_change_kc
import rospy
from sensor_msgs.msg import JointState


class Transformer:
	#stream index用来标注是哪个相机，左为0 
	# of the zeroth (left) camera
	# stream_index 

	# self.pose = np.array([0.0, 0.0, 0.0])
	# def __init__(self):
	def __init__(self,name):

		#init a miro camera model
		self.cam = miro.lib.camera_model.CameraModel()
		self.cam.set_frame_size(640, 360)
		self.kc = miro.lib.kc_interf.kc_miro()
		self.kc_updater = node_change_kc.ChangeKc(name)
		self.kin_sub = rospy.Subscriber(name + "sensors/kinematic_joints", JointState,self.callback_kin)
		self.kinematic_joints = JointState()

	def callback_kin(self,msg):
		self.kinematic_joints =msg

	def pixel_to_world(self,stream_index,pixel,pose):

		#every time, if we want to change frames, we have to update the 
		# #status of the kinematic chains
		# print("kinematic_joints.position------->",self.kinematic_joints.position)
		# print("kinematic_joints.position------->",np.array(self.kinematic_joints.position))
		# self.kc_updater.change_kc(np.array(self.kinematic_joints.position))
		kinematic_joints = np.array(self.kinematic_joints.position)
		self.kc.setConfig(kinematic_joints)
		#这个range不知道是怎么求解，需要问下。
		# and a presumed range
		r = 0.5
			
		# map to a view line in CAM
		v = self.cam.p2v(pixel)

		# map to a location in HEAD
		oh = self.cam.v2oh(stream_index, v, r)
		
		# and use kc to map that into FOOT
		of = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_FOOT, oh)
		
		# finally, if we'd been integrating the speed so that we had a
		# current estimate of the robot position, we could...
		# ...oh, wait, we have!
		self.kc.setPose(pose)
		
		# so we can, finally, map the target into WORLD
		ow = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, oh)
		# print "target in FOOT", of
		# print "target in WORLD", ow
		return ow