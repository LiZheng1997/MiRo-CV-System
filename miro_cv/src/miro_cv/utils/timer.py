import time
import datetime
import rospy

# t = rospy.Time.from_sec(time.time())
# seconds = t.to_sec() #floating point
# # nanoseconds = t.to_nsec()
# print seconds

# d = rospy.Duration.from_sec(60.1) # a minute and change
# seconds = d.to_sec() #floating point
# print seconds
# theTime = datetime.datetime.now()
# time_start=time.time()
# time_end=time.time()
# print time_start
# print("time cost",time_end-time_start)
rospy.init_node( "_client_demo")
# old_time = datetime.datetime.now()
# old_time = old_time.second
old_time =  rospy.get_rostime()
print "secs",old_time.secs
# old_time = old_ti
# for i in range(1000000):
#     print i
current_time = rospy.get_rostime()
# current_time = current_time.second
print (current_time - old_time)