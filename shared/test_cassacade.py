# -*- coding: utf-8 -*-
#!/usr/bin/python
import cv2
import numpy as np
import time
hog = cv2.HOGDescriptor()
hog.load('myHogDector.bin')
cap = cv2.VideoCapture(1)
if cap.isOpened() == False:
	cap.open()
else: 
	print("打开成功")
	cap.get(3)
	# ret = cap.set(3320)
	# ret = cap.set(4240)

while True:
	
	time.sleep(0.1)
	ok, img = cap.read()

	
	rects, wei = hog.detectMultiScale(img, winStride=(4, 4),padding=(8, 8), scale=1.05)
	for (x, y, w, h) in rects:
		cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
		#在这里进行像素坐标的转换 转换到世界坐标中，我们要的是这个物体在MIRO的相对空间中
		#的距离和速度的信息，这里可以检测到人体之后，或得到的是人体的图框，
	cv2.imshow('a', img)
	if cv2.waitKey(1)&0xff == 27:    # esc键
		break
cap.release()
cv2.destroyAllWindows()