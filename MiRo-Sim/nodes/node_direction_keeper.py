# -*- coding: utf-8 -*-
from node_detector import * 
from utils import data_resolver
import miro2 as miro
import rospy
from geometry_msgs.msg import TwistStamped
from transform import * 
import numpy as np
import time
import math

class DirectionKeeper:

    def __init__(self,name):
        # topic root
        self.topic_root = name
        self.velocity = TwistStamped()
        # self.detector = Detector()
        self.data_resolver = data_resolver.DataResolver()
        self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel",
                                                TwistStamped, queue_size=1)
        #self.cam_model = miro.utils.CameraModel()
        self.transformer = Transformer(name)
        self.active = True

    #这里存在一个隐形的问题，我这里设计的时候是希望，从detection的5帧画面中计算一个平均的发现的物体的
    #中心点的值，但是因为detection的精度不高的原因，这里可能存在严重的误差，所以尽量将精度提高，同时，
    # 精度高的情况下，可以不进行平均求中心点的操作。使用其他的CNN 高精度模型，可以不用这样操作，提高
    #反应速度。
    #Using some simple math calculation to control the direction, this function
    #can get a reguralized range, the volume of the range is the number of pixels,
    #if the direction of the robot lies on the sight line, the funcion will return a 
    #flag named is_directed, then the direction need not to be corrected.

    def diret_keeper(self,avg_l, avg_r,pose):
        '''direction keeping control for robots in both left and right sides.
            params:
                avg_l: average left center pos values
                avg_r: average right center pos values
                pose: current robot pose in world coord
        '''
        # reg_range = 0  # use the reg_range to direct the robot to the sight line
        direction = None
        angle = 0
        t_pos = None
        beta = 0
        # set the width to the default value 640 默认的设定是图像的宽是640
        dif_l = 640 - avg_l[0]  # + 50
        center_pos = [460, avg_l[1]]  # the center pos x is 460
        if dif_l <= (180 + 2 ) and dif_l >= (180 - 2):
            print("direction is corrected.")
            direction = 0
            angle, beta, t_pos= self.direct_to_sightline(center_pos, avg_r, 1, pose)
        elif dif_l < (180 - 2):
            direction = -1
            # reg_range = int(abs(dif_l - 180) )
            # print("reg_range-------------->",reg_range)
            # center_pos[0] = avg_l[0] - reg_range  #35 50
            angle, beta, t_pos = self.direct_to_sightline(center_pos, avg_r, 1, pose)

        elif dif_l > (180 + 2):
            direction  = 1
            # reg_range = int(abs(dif_l - 180) )
            # print("reg_range-------------->",reg_range)
            # center_pos[0] = avg_l[0] + reg_range
            angle, beta, t_pos = self.direct_to_sightline(center_pos, avg_r, 1, pose)
        return angle, beta, t_pos, direction, center_pos


    #这里使用的是单目的测距方式，但是MIRO的相机存在他独有的特性，不是在一个水平几何平面上，所以，
    # 他的相机的中心点之间的误差比较大，越远物体实际的距离越大，近小远大。这里的角度应该给上一个固定的
    # 补充（增益值）gain value， 不然实际使用的是相机的中心点，然后我们需要的是两个相机的焦点的中心点，
    # 也就是使得MIRO可以正中对准target。
    # 	
    #This function use the pixel_to_world method from the transform class, through using
    #pixel_to_world function, we can transform the coordination of the target to the 
    #world coordination, in this case, we can calculate the angle to be corrected for 
    #obtaining the angular velocity.
    #加上反向的一个角度增益
    def diret_keeper_s (self, avg, cam_index ,pose):
        '''direction keeping control for robots in single sides.
            params:
                avg: average angle value.
                cam_index: camera index, 0 for left camera, 1 for right camera.
                pose: current robot pose in world coord
            return:
                angle: 
                beta:
        '''
        angle = 0
        beta = 0
        t_pos =None
        direction = None

        # if the camera index is 0, the head center is around the X axis of 460 pixels
        # if the camera index is 1, the head center is around the X axis of 180 pixels
        # so if we want the robot's head can direct point to the target's center, we need
        # to use the 460(X axis) and 180(X axis) for left camera and right camera repectively.
        # the center pos means the target position appears on image which can drive the robot's
        # head point direct towards the target.
        if cam_index == 0 :
            center_pos = [460,avg[1]]  # why 460?, because the center_pos X value is not the head center.
        elif cam_index == 1:
            center_pos = [180,avg[1]]  # why 180?, because the center_pos X value is not the head center.
        if avg[0] < center_pos[0] - 2:
            direction = 1
            angle, beta, t_pos= self.direct_to_sightline(center_pos,avg,cam_index, pose)
        elif center_pos[0]- 2 <= avg[0] and  avg[0]  <= center_pos[0]+ 2:
            print("direction is corrected.")
            direction = 0
            angle, beta, t_pos= self.direct_to_sightline(center_pos,avg,cam_index, pose)
        elif avg[0]> center_pos[0]+ 2:
            direction = -1
            angle,beta, t_pos = self.direct_to_sightline(center_pos,avg,cam_index, pose)
        return angle, beta, t_pos, direction, center_pos

    #时刻注意这里的角度的求解，利用世界坐标的2D平面进行几何关系的计算，所以这里我们的姿态需要不断的
    #更新，在现实世界中，机器人的pose计算需要一些更加复杂准确的算法，涉及到里程计的计算，计算行走的
    #距离和旋转的总角度，（x,y, theta)
    # remeber the pose here in the function should be changed during the moving of 
    # the robot, so the pose later will be a parameter in the function.
    def direct_to_sightline(self, center_pos, target_pos, cam_index, pose):
        '''This func is the core func to control robots to minize the angle value
        between its head direction and the target direction.
        '''

        # assume that the pose of miro robot is at the original point of
        # the world coordination system
        robot_pos = np.array([pose[0],pose[1]])
        c_pos = self.transformer.pixel_to_world(cam_index, center_pos,pose )
        t_pos = self.transformer.pixel_to_world(cam_index, target_pos,pose )

        # print("c_pos:",c_pos[0])
        # print("t_pos:",t_pos[0])
        # print("robot_pos:",robot_pos)
        #cr_distance is the distace  between target and the robot
        # tr_distance is the distance between center and the robot
        cr_distance = self.data_resolver.distance_resolver(c_pos,robot_pos )
        cr_y_distance = abs(c_pos[0]- robot_pos[0])

        tr_distance = self.data_resolver.distance_resolver(t_pos,robot_pos )
        tr_y__distance = abs(t_pos[0] - robot_pos[0])
        # print("c_pos:",c_pos)
        # print("t_pos:",t_pos)
        # print("cr_distance", cr_distance)
        print("tr_distance---------------------------------->",tr_distance)
        # print("cr_y_distance", cr_y_distance)
        # print("tr_y__distance", tr_y__distance)
        #the angle of can be obtained through geomtery methods
        alpha = self.data_resolver.angle_resolver(cr_y_distance, cr_distance, "sin")
        beta = self.data_resolver.angle_resolver(tr_y__distance, tr_distance, "sin")
        theta = abs(alpha - beta)
        #机器人修正角度为theta， target和robot之间的lambda为 beta这里
        # ，alpha为机器人的中心旋转角。用beta - alpha的原因和驱动轮子的速度
        #是正是负，左转还是右转有关。
        return theta, beta, t_pos

    def reguralize_angle(self, angle, direction, slow):
        '''reguralize the robot's augular velocity refer to angle values.
            params:
                angle: the angle value needs to be minized.
                direction: the direction stands for left(negative value) and right.
                slow: the slow mode to choose or not, True is slow mode.
        '''
        if slow:
            spin = 3.141592 * 0.1  # 6.2832 is two pi(3.141592), 0.1 is the speed gain.
            # spin = math.radians(angle) *4
            v = angle/90 #using the

        else:
            spin = 3.141592 * 0.8  # 6.2832 is two pi(3.141592), 0.7 is the speed gain.
            # spin = math.radians(angle) * 8
            v = angle/90

        if direction == 1:
            self.velocity.twist.linear.x = 0.4  # linear speed at x axis, head towards.
            self.velocity.twist.angular.z = direction * spin * v
            # self.velocity.twist.angular.z = math.radians(angle)
            self.velocity_pub.publish(self.velocity)

        elif direction == -1:
            self.velocity.twist.linear.x = 0.4
            self.velocity.twist.angular.z = direction * spin * v
            # self.velocity.twist.angular.z = math.radians(-angle)
            self.velocity_pub.publish(self.velocity)

        elif direction == 0:
            self.velocity.twist.linear.x = 0.4
            self.velocity.twist.angular.z = direction * spin * v
            # self.velocity.twist.angular.z = math.radians(-angle)
            self.velocity_pub.publish(self.velocity)
        return self.velocity
