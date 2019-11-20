#!/usr/bin/env python

import sys
import time
import math
from threading import Thread
import os

import miro2 as miro

import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

#Initialisation
rospy.init_node("custom_controller")
#topic_root = '/' + os.getenv("MIRO_ROBOT_NAME") + '/'
topic_root = '/' + 'miro_0' + '/'

velocity_pub = rospy.Publisher(topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
cosmetic_joints_pub = rospy.Publisher(topic_root + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
illum_pub = rospy.Publisher(topic_root + "control/illum", UInt32MultiArray, queue_size=0)
kinematic_joints_pub = rospy.Publisher(topic_root + "control/kinematic_joints", JointState, queue_size=0)

velocity = TwistStamped()

kin_joints = JointState()
kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

cos_joints = Float32MultiArray()
cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

illum = UInt32MultiArray()
illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

yaw_margin = 0.12

lift_current = None
pitch_current = None
yaw_current = None
update_flag = False

def update_kin(msg):
    global lift_current
    global pitch_current
    global yaw_current
    global update_flag
    lift_current = msg.position[miro.constants.JOINT_LIFT]
    pitch_current = msg.position[miro.constants.JOINT_PITCH]
    yaw_current = msg.position[miro.constants.JOINT_YAW]
    update_flag = True

kin_sub = rospy.Subscriber(topic_root + "sensors/kinematic_joints", JointState, update_kin)

def wag_tail_func(dur, rate):
    timeout_start = time.time()
    wag_phase = 0
    while time.time() < timeout_start + dur:
        wag_phase += math.pi / rate
        if wag_phase >= 2 * math.pi:
            wag_phase -= 2 * math.pi
        cos_joints.data[miro.constants.JOINT_WAG] = math.sin(wag_phase) * 0.5 + 0.5
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.1)

def blink_func():
    cos_joints.data[miro.constants.JOINT_EYE_L] = 1.0
    cos_joints.data[miro.constants.JOINT_EYE_R] = 1.0
    cosmetic_joints_pub.publish(cos_joints)
    time.sleep(0.2)
    cos_joints.data[miro.constants.JOINT_EYE_L] = 0.0
    cos_joints.data[miro.constants.JOINT_EYE_R] = 0.0
    cosmetic_joints_pub.publish(cos_joints)
    time.sleep(0.2)

#This code was written at 2am, it is not of high quality, it may not even make sense, it works.... just!

#Main
while True:
    loop = True
    input = sys.stdin.readline()

    if input == "0\n":
        kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
        illum_pub.publish(illum)
        cosmetic_joints_pub.publish(cos_joints)
        kinematic_joints_pub.publish(kin_joints)

    elif input == "1\n":
        print("Completeting Action 1")
        t = 0.0
        while t <= 5:
            colour = 0x00000099
            speed_scalar = 0.5
            multiplier = abs(math.cos(math.pi * t * speed_scalar))
            brightness = int(round(255 * multiplier)) << 24
            colour = colour | brightness
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            t = t + 0.02
            time.sleep(0.02)

    elif input == "2\n":
        print("Completeting Action 2")
        kin_joints.position[miro.constants.JOINT_YAW] = math.radians(-60)
        kinematic_joints_pub.publish(kin_joints)
        time.sleep(1)
        cos_joints.data[miro.constants.JOINT_EAR_L] = 0.0
        cos_joints.data[miro.constants.JOINT_EAR_R] = 0.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.3)
        cos_joints.data[miro.constants.JOINT_EAR_L] = 1.0
        cos_joints.data[miro.constants.JOINT_EAR_R] = 1.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.3)
        cos_joints.data[miro.constants.JOINT_EAR_L] = 0.0
        cos_joints.data[miro.constants.JOINT_EAR_R] = 0.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.3)
        cos_joints.data[miro.constants.JOINT_EAR_L] = 1.0
        cos_joints.data[miro.constants.JOINT_EAR_R] = 1.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.3)
        cos_joints.data[miro.constants.JOINT_EAR_L] = 0.5
        cos_joints.data[miro.constants.JOINT_EAR_R] = 0.5
        cosmetic_joints_pub.publish(cos_joints)

    elif input == "3\n":
        print("Completing Action 3")
        t_now = 0.0
        yaw_pos = 0
        lift_pos = math.radians(34)
        lift_des = math.radians(5)
        lift_dif = lift_des - lift_pos
        while t_now < 1:
            lift_now = lift_pos + math.sin(t_now * 0.5 * math.pi) * lift_dif
            kin_joints.position[miro.constants.JOINT_LIFT] = lift_now
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        t_now = 0.0
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        time.sleep(1)
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        while t_now < 3:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = -math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)

    elif input == "4\n":
        print("Completing Action 4")
        t_now = 0.0
        yaw_pos = math.radians(60.0)
        kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
        kinematic_joints_pub.publish(kin_joints)
        time.sleep(0.6)
        cos_joints.data[miro.constants.JOINT_EYE_L] = 1.0
        cos_joints.data[miro.constants.JOINT_EYE_R] = 1.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.2)
        cos_joints.data[miro.constants.JOINT_EYE_L] = 0.0
        cos_joints.data[miro.constants.JOINT_EYE_R] = 0.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.2)
        cos_joints.data[miro.constants.JOINT_EYE_L] = 1.0
        cos_joints.data[miro.constants.JOINT_EYE_R] = 1.0
        cosmetic_joints_pub.publish(cos_joints)
        time.sleep(0.2)
        cos_joints.data[miro.constants.JOINT_EYE_L] = 0.0
        cos_joints.data[miro.constants.JOINT_EYE_R] = 0.0
        cosmetic_joints_pub.publish(cos_joints)

    elif input == "5\n":
        print("Completing Action 5")
        t_now = 0.0
        illum.data = [0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF]
        illum_pub.publish(illum)
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(-30.0)
            pitch_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(8.0)
            lift_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(57)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kin_joints.position[miro.constants.JOINT_LIFT] = lift_pos
            eye = math.sin(t_now * 0.25 * math.pi) * 0.5
            ear = math.sin(t_now * 0.25 * math.pi) * 1.0
            cos_joints.data[miro.constants.JOINT_EYE_L] = eye
            cos_joints.data[miro.constants.JOINT_EYE_R] = eye
            cos_joints.data[miro.constants.JOINT_EAR_L] = ear
            cos_joints.data[miro.constants.JOINT_EAR_R] = ear
            cosmetic_joints_pub.publish(cos_joints)
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)

    elif input == "6\n":
        print("Completing Action 6")

        yaw_pos = math.radians(60)
        kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
        kinematic_joints_pub.publish(kin_joints)
        time.sleep(2)

        velocity.twist.linear.x = 0.3
        velocity.twist.angular.z = math.radians(100)
        t_now = 0.0
        while t_now < 1:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

        velocity.twist.angular.z = 0
        t_now = 0.0
        while t_now < 2:
            yaw_pos = yaw_pos + math.radians(-120)/(2/0.01)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

        while t_now < 5:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input == "7\n":
        print("Completing Action 7")
        t_now = 0.0
        yaw_pos = 0
        velocity.twist.linear.x = 0.3
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            t_now+=0.01
            time.sleep(0.01)
        s_now = 0.0
        while s_now < 2:
            velocity_pub.publish(velocity)
            s_now += 0.01
            time.sleep(0.01)
        print("wait 1")
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            t_now+=0.01
            time.sleep(0.01)
        while t_now < 3:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = -math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            t_now+=0.01
            time.sleep(0.01)

        s_now = 0.0
        while s_now < 2:
            velocity_pub.publish(velocity)
            s_now += 0.01
            time.sleep(0.01)

        while t_now < 4:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = -math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            t_now+=0.01
            time.sleep(0.01)

        s_now = 0.0
        while s_now < 2:
            velocity_pub.publish(velocity)
            s_now += 0.01
            time.sleep(0.01)

    elif input =="8\n":
        print("Completing Action 8")
        velocity.twist.linear.x = 0.2
        t_now = 0.0
        while t_now < 50:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="9\n":
        print("Completing Action 9")
        velocity.twist.linear.x = 0.4
        t_now = 0.0
        while t_now < 20:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="10\n":
        print("Completing Action 10")
        velocity.twist.linear.x = 0.1
        illum.data = [0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF]
        illum_pub.publish(illum)
        t_now = 0.0
        while t_now < 20:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="11\n":
        print("Completing Action 11")
        wag_thread = Thread(target=wag_tail_func, args=(10, 3))
        wag_thread.start()
        velocity.twist.linear.x = 0.3
        t_now = 0.0
        while t_now < 10:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="12\n":
        print("Completing Action 12")
        t_now = 0.0
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            lift_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(5)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        wag_thread = Thread(target=wag_tail_func, args=(10, 3))
        wag_thread.start()
        time.sleep(10)

    elif input =="13\n":
        print("Completing Action 13")
        t_now = 0.0
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            lift_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(5)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kin_joints.position[miro.constants.JOINT_LIFT] = lift_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        time.sleep(3)
        velocity.twist.linear.x = 0.2
        t_now = 0.0
        while t_now < 10:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01
        t_now = 1.0
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            velocity_pub.publish(velocity)
            t_now+=0.01
            time.sleep(0.01)
        t_now = 0.0
        while t_now < 2:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="14\n":
        print("Completing Action 14")
        velocity.twist.linear.x = 0.4
        velocity.twist.angular.z = math.radians(100)
        t_now = 0.0
        t = 0.0
        while t_now < 20:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01
            t = t + 0.01

    elif input =="15\n":
        print("Completing Action 15")
        wag_thread = Thread(target=wag_tail_func, args=(10, 5))
        wag_thread.start()
        cos_joints.data[miro.constants.JOINT_EYE_L] = 0.5
        cos_joints.data[miro.constants.JOINT_EYE_R] = 0.5
        cosmetic_joints_pub.publish(cos_joints)
        t_now=0.0
        t = 0
        while t_now < 10:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            time.sleep(0.01)
            t_now+=0.01
            t = t + 0.01

    elif input =="16a\n":
        print("Completing Action 16a")
        velocity.twist.angular.z = math.radians(200)
        velocity.twist.linear.x = 0
        t_now = 0.0
        while t_now < 1:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

        velocity.twist.angular.z = 0
        velocity.twist.linear.x = 0.3
        t_now = 0.0
        while t_now < 2.5:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

        t_now = 0.0
        illum.data = [0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF, 0xFF0000FF]
        illum_pub.publish(illum)
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(-30.0)
            pitch_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(8.0)
            lift_pos = math.sin(t_now * 0.25 * math.pi) * math.radians(57)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kin_joints.position[miro.constants.JOINT_LIFT] = lift_pos
            eye = math.sin(t_now * 0.25 * math.pi) * 0.5
            ear = math.sin(t_now * 0.25 * math.pi) * 1.0
            cos_joints.data[miro.constants.JOINT_EYE_L] = eye
            cos_joints.data[miro.constants.JOINT_EYE_R] = eye
            cos_joints.data[miro.constants.JOINT_EAR_L] = ear
            cos_joints.data[miro.constants.JOINT_EAR_R] = ear
            cosmetic_joints_pub.publish(cos_joints)
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)

    elif input =="16b\n":
        print("Completing Action 16b")
        t_now = 0.0
        illum.data = [0xFFFFFFFF]*6
        illum_pub.publish(illum)
        kin_joints.position[miro.constants.JOINT_YAW] = 0
        kin_joints.position[miro.constants.JOINT_PITCH] = 0
        kin_joints.position[miro.constants.JOINT_LIFT] = math.radians(34)
        cos_joints.data[miro.constants.JOINT_EYE_L] = 0
        cos_joints.data[miro.constants.JOINT_EYE_R] = 0
        cos_joints.data[miro.constants.JOINT_EAR_L] = 0
        cos_joints.data[miro.constants.JOINT_EAR_R] = 0
        cosmetic_joints_pub.publish(cos_joints)
        kinematic_joints_pub.publish(kin_joints)

        time.sleep(1)

        velocity.twist.angular.z = math.radians(200)
        velocity.twist.linear.x = 0
        t_now = 0.0
        while t_now < 1:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

        velocity.twist.angular.z = 0
        velocity.twist.linear.x = 0.3
        t_now = 0.0
        while t_now < 2:
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="17\n":
        print("Completing Action 17")
        velocity.twist.angular.z = math.radians(-15)
        velocity.twist.linear.x = 0.3
        t_now = 0.0
        t = 0
        while t_now < 10:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01
            t+=0.01

    elif input =="18\n":
        print("Completing Action 18")
        velocity.twist.angular.z = 0
        velocity.twist.linear.x = 0.3
        t_now = 0.0
        t=0
        while t_now < 5:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01


        t_now = 0.0
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        while t_now < 2:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        while t_now < 3:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = -math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)

        velocity.twist.angular.z = math.radians(-200)
        velocity.twist.linear.x = 0
        t_now = 0.0
        while t_now < 10:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="19\n":
        print("Completing Action 19")
        velocity.twist.angular.z = math.radians(200)
        velocity.twist.linear.x = 0
        t_now = 0.0
        while t_now < 10:
            green = 0x00009900
            yellow = 0x00999900
            multiplier = abs(math.sin(math.pi * t_now))
            brightness = int(round(255 * multiplier)) << 24
            if t <= 1.0:
                colour = green | brightness
            elif 1.0 < t <= 2.0:
                colour = yellow | brightness
            else:
                t = 0
            t+=0.01
            for i in range(6):
                illum.data[i] = colour
            illum_pub.publish(illum)
            velocity_pub.publish(velocity)
            time.sleep(0.01)
            t_now+=0.01

    elif input =="20\n":
        print("Completing Action 20")
        t_now = 0.0
        while t_now < 1:
            yaw_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(60.0)
            pitch_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(-22.0)
            lift_pos = math.sin(t_now * 0.5 * math.pi) * math.radians(5)
            kin_joints.position[miro.constants.JOINT_YAW] = yaw_pos
            kin_joints.position[miro.constants.JOINT_PITCH] = pitch_pos
            kinematic_joints_pub.publish(kin_joints)
            t_now+=0.01
            time.sleep(0.01)
        wag_thread = Thread(target=wag_tail_func, args=(10, 3))
        wag_thread.start()
        time.sleep(10)


    elif input =="q\n":
        break
    else:
        print("Command not recognised")
