#!/usr/bin/python3
# -*- coding:utf-8 -*-
#
#   @section COPYRIGHT
#   Copyright (C) 2023 Neurons Vision Ltd
#
#   @section AUTHOR
#   Neuros Vision https://www.neuronsvision.com
#
#   @section LICENSE
#
#
#
#   #############################################################################
#   # This is the core version of my previous dissertation project.             #
#   # This project is designed for off-board deployment or the simulator.       #
#   # If you want to reproduce my project result, you can clone this            #
#   # core branch, this can be easier and faster for you to implement.          #
#   # All codes are deployed off-board rather than on-board, which means all    #
#   # codes will be deployed on a local computer rather than on a miro robot.   #
#   # Functionalities include object detection and object tracking, then to     #
#   # the control module for following a target and also intercepting a target  #
#   # can be implemented.                                                       #
#   # Some codes are inspired from source codes in the MiRo project. You can    #
#   # you refer to Consequential Robotics http://consequentialrobotics.com      #
#   #############################################################################


import numpy as np
from std_msgs.msg import UInt16

class DataResolver:
    '''This class is designed for resolving all data relevant processing.
    attribute:
        arr: a array to store bboxes
    '''

    def __init__(self):
        self.arr = []

    # 计算所有的数组中除去nan值的平均数
    def avg_resolver(self, arr):
        '''This function is mainly designed for calculating the avearage value of all nums.
        attribute:
            arr: a array taking out of nan values.
            cal_avg: a instance of calculating the avg value.
        '''
        self.arr = arr [np.logical_not(np.isnan(arr))]  # pick out all nan values
        avg = self.cal_avg(self.arr)

        return avg

    def cal_avg(self,arr):
        '''This func is designed for calculating the avg value of all nums. 
        '''
        if arr is not None:
            avg = []
            [row] = arr.shape
            x_arr = np.array([],dtype=UInt16)
            y_arr = np.array([],dtype=UInt16)
            for i in range(row):
                if i % 2 == 0:
                    x_arr = np.append(x_arr,arr[i] )
                else:
                    y_arr = np.append(y_arr,arr[i] )
        try:
            avg_x = np.mean(x_arr)
            avg_y = np.mean(y_arr)
            avg =[avg_x, avg_y]
        except ZeroDivisionError as e :
            print("no targets have been found!")
            print(e)

        else:
            pass

        return avg

    def box_resolver(self, bbox_lst):
        '''This func is for calculating all bbox elements' avg values.

        sumx, sumy, sumw, sumh are sum of all bboxes' [x,y,w,h] seperately.
        '''
        sum_x, sum_y ,sum_w,sum_h = 0,0,0,0
        num = len(bbox_lst)

        if num == 0:
            print("There is no data in the bbox_lst.")
            # pass

        else:
            for i in range(num):
                box_tuple = bbox_lst[i]
                sum_x += box_tuple[0]
                sum_y += box_tuple[1]
                sum_w += box_tuple[2]
                sum_h += box_tuple[3]
            x = sum_x/num
            y = sum_y/num
            w = sum_w/num
            h = sum_h/num
            box_tuple = (x,y,w,h)
        return box_tuple

    def distance_resolver(self,start, end):
        '''This func is designed for calculating the distance in the world coordination.
        '''
        x_distance = np.square(start[0] - end[0])
        y_distance = np.square(start[1]- end[1])
        distance = np.sqrt((x_distance + y_distance))
        return distance

    # this function can change angle to degrees
    def angle_resolver(self, a, b, angle_type):
        '''This function can change angle to degrees
        '''
        if angle_type == "sin":
            sin = a / b
            if sin > 1:
                sin = 1
            inv = np.arcsin(sin)
            angle = np.degrees(inv)

        elif angle_type == "cos":
            cos = a/b
            if cos > 1:
                cos = 1
            inv = np.arccos(cos)
            angle = np.degrees(inv)

        elif angle_type == "tan":
            tan = a/b
            if tan >1:
                tan=1
            inv = np.arctan(tan)
            angle = np.degrees(inv)

        return angle
