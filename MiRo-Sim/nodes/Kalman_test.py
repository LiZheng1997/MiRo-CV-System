import cv2
import numpy as np
import matplotlib.pyplot as plt

frame = np.zeros((800,800,3),np.uint8)
last_mes = current_mes = np.array((2,1),np.float32)
last_pre = current_pre = np.array((2,1),np.float32)

def mousemove(event, x,y,s,p):
    global frame, current_mes, mes, last_mes, current_pre, last_pre
    last_pre = current_pre
    last_mes = current_mes
    current_mes = np.array([[np.float32(x)],[np.float32(y)]])

    kalman.correct(current_mes)
    current_pre = kalman.predict()

    lmx, lmy = last_mes[0],last_mes[1]
    lpx, lpy = last_pre[0],last_pre[1]
    cmx, cmy = current_mes[0],current_mes[1]    
    cpx, cpy = current_pre[0],current_pre[1]    
    cv2.line(frame, (lmx,lmy),(cmx,cmy),(0,200,0))
    cv2.line(frame, (lpx,lpy),(cpx,cpy),(0,0,200))


cv2.namedWindow("Kalman")
cv2.setMouseCallback("Kalman", mousemove)
kalman = cv2.KalmanFilter(4,2)
kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 0.003
kalman.measurementNoiseCov = np.array([[1,0],[0,1]], np.float32) * 1

while(True):
    cv2.imshow('Kalman',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()