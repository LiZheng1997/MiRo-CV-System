# -*- coding: utf-8 -*-
#!/usr/bin/python
#当前demo进行MIRO的移动，移动到指定的位置，完成截取任务。


#kc的一些基本值的初始化，
# function to initialise default values
def kc_init(tick, flags, link):
	global KC_TICK_SEC, KC_PUSH_FLAGS_DEFAULT, KC_PUSH_LINK_DEFAULT
	KC_TICK_SEC=tick
	KC_PUSH_FLAGS_DEFAULT=flags 
	KC_PUSH_LINK_DEFAULT=link


def __init__(self):

	self.flags = KC_PUSH_FLAGS_DEFAULT
	self.link = KC_PUSH_LINK_DEFAULT
	self.pos = np.array([0.0, 0.0, 0.0])
	self.vec = np.array([0.0, 0.0, 0.0])
	#
	self.clock = ActionClock()

	self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)

def move_to_target(self):
	# compute start point for fovea in WORLD
	self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
	#计算终止点在世界坐标中的具体位置。这里是正向的计算。反向中，需要修改
	# compute end point for fovea in WORLD
	fovea_f_WORLD = self.kc.changeFrameAbs(
			miro.constants.LINK_HEAD,
			miro.constants.LINK_WORLD,
			miro.utils.kc_interf.kc_view_to_HEAD(
				self.input.priority_peak.azim,
				self.input.priority_peak.elev,
				self.input.priority_peak.range
				)
			)

	#限制终止点的范围 是可以到达的，不过这里给的是声呐的范围，。。
	# limit end point to be reachable
	fovea_f_WORLD[2] = np.clip(fovea_f_WORLD[2],
						self.pars.geom.reachable_z_min,
						self.pars.geom.reachable_z_max
						)

	#计算出来的distance，在世界坐标中的位移。
	# compute total movement fovea will make in world
	self.dfovea_WORLD = fovea_f_WORLD - self.fovea_i_WORLD


	# read clock
	x = self.clock.cosine_profile()
	self.clock.advance(True)

	# compute an interim target along a straight trajectory
	fovea_x_WORLD = x * self.dfovea_WORLD + self.fovea_i_WORLD

	# transform interim target into HEAD for actioning
	fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, fovea_x_WORLD)

	# 这个push函数在哪
	# apply push
	self.apply_push_fovea(fovea_x_HEAD - self.fovea_HEAD)


	# debug fovea movement through WORLD
	#fovea_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
	#print "fovea_WORLD", fovea_WORLD, self.clock.toString()


	def push_to_point(self, point):
		self.pause()
		push_msg = miro.msg.push()
		push_loc = miro.utils.get("LOC_SONAR_FOVEA_HEAD")
		push_msg.pushpos = Vector3(push_loc[0], push_loc[1], push_loc[2])
		push_vec = point - push_loc
		push_vec /= np.linalg.norm(push_vec)
		push_vec *= 0.1
		push_msg.pushvec = Vector3(push_vec[0], push_vec[1], push_vec[2])
		push_msg.flags = miro.constants.PUSH_FLAG_IMPULSE
		push_msg.link = miro.constants.LINK_HEAD
		self.push_pub.publish(push_msg)

