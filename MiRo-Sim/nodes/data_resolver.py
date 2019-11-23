import numpy as np
# import pandas as pd 
from std_msgs.msg import UInt16

class DataResolver:

    def __init__(self):
        
        self.arr = []

    def avg_resolver(self, arr):
        #pick out all none values
        self.arr = arr [np.logical_not(np.isnan(arr))]
        # self.r_arr = right_arr [np.logical_not(pd.isnull(right_arr))]
        avg = self.cal_avg(self.arr)
        return avg

    def cal_avg(self,arr):
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
            # print("avg",avg)
            # count =0
        else:
            pass
        return avg

    def box_resolver(self, bbox_lst):
        sum_x, sum_y ,sum_w,sum_h = 0,0,0,0
        num = len(bbox_lst)
        if num == 0:
            print("There is no data in the bbox_lst.")
            pass
        else:
            # sum_x = np.sum()
            for i in range(num):
                box_tuple = bbox_lst[i]
                # print("box:",box_tuple)
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

    #This function can calculate the distance in the world coordination.
    def distance_resolver(self,start, end):
        x_distance = np.square(start[0] - end[0])
        y_distance = np.square(start[1]- end[1])
        distance = np.sqrt((x_distance + y_distance))
        return distance

    def angle_resolver(self, a, b, angle_type):

        if angle_type == "sin":
      
            sin = a / b
            if sin > 1:
                sin = 1
            # print("sin:",sin)
            inv = np.arcsin(sin)
            angle = np.degrees(inv)
            # print("the volume of degrees need to be corrected:",angle)
        elif angle_type == "cos":
            cos = a/ b 
            if cos > 1:
                cos = 1
            # print("cos:", cos)
            inv = np.arccos(cos)
            angle = np.degrees(inv)
            # print("the volume of degrees need to be corrected:",angle)
        elif angle_type == "tan":
            tan = a/b
            if tan >1:
                tan=1
            inv = np.arctan(tan)
            angle = np.degrees(inv)
            # print("the volume of degrees need to be corrected:",angle)
        return angle