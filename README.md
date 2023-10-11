# MiRo-CV-System

This is my first version of an autonomous visual system which is purely based on using OpenCV APIs, and the final aim of this project is to intercepting another moving target. I defined three targets, named pedestrians, another MiRo robot and MiRo toy ball in this project. There are three scenarios for different deploying methods. One is deploying on-board, and the other two are deploying off-board or in simulator (Gazebo).  Some codes are inspired from code samples in the MDK-2019 version. Thanks for their brilliant work on building MiRo robots. I am also one of the big fan loving biomimetic robots.



## Background

At the begining, this project is my Master's dissertation project in 2019, now I uploaded all codes and more improvements will be implemneted in the future. All relevant resources will be uploaded, including training sample images and model files.



## Methods

This CV system is mainly based on OpenCV library, which uses some conventional computer vision algorithms, like Hough circle detection, Gaussian filter, Median filter, Optical flow estimation and stereo camera depth calculation, etc. Also, DNN models are used in this project to achieve real time detection through Depth Learning technologies.  



### Detector Node

#### MiRo Detection Module

which utilizes the detection module based on CascadeClassifier, the classifier model is trained using collected indoor MiRo robot data. Positive sample is a MiRo robot in the image, negative sample is some other things in indoor environment.

Hence, this cascade classifier needs to judge whether there is a MiRo robot in images. 



#### Ball Detection Module

This module is based on the Hough circle detection in opencv lib. Therefore, this module is tend to use in a Gazebo simulator, as the real world environment is too noisy for this algorithm, and the effect of detection is unstable. 



#### Pedestrian Detection Module

This module is based on the dnn module in opencv lib. Using some traditional CNN model, like darknet, faster-rcnn and yolo, to achieve pedestrian detection in real world with considerable latency. 



### Tracker Node

#### Single-Tracking Module

This module is based on the tracker api in opencv. There are 8 trackers in total after opencv3.4 version. So, if you want to use this project, you have to use a higher version than opecnv-3.4, or some unexpected errors will be produced. TLD tracker is the one I prefer to use in indoor environment with occlusion situation.

### Multi-Tracking Module

This module is based on the multitracker api in opencv.



### Control Node

Control node contains a direction keeper node, this is the main function for maintaining the direction of a MiRo robot. 

#### Orientation Control Module

This module is based on a designed orientation correction algorithms. Orientation errors is calculated based on the angle variation  between the viewline and the ego-to-target line. Hence, MiRo robot will be driven to point its head towards the target and keep following the target, then it can intercept this moving target with considerable higher linear velocity. Angular velocity will be corrected based on the angle variation mentioned before. 

#### Safety Controller Module

This module is based on the sonar sensor on the MiRo's nose. I use the data from sonar sensor to estimate the distance between the target and the MiRo. This is not accurate enough in real time tests. 



### Path Planning Node

#### Path planning Module

Global path planning and Local path planning are designed for intercepting another moving target. Kinematic model is necessary for this module to work smoothly in real time.



#### Kalman Filter Pose Estimation Module

Using last second pose and utilize Kalman Filter to predict the current ego pose. 



## Summary

All codes are implemented based on doing self-learning roughly in 5 months. Necessary knowledge includes ROS, Computer Vision basic theories, OpenCV APIs, Physical Robots' experiments, Linux system, Machine Learning, Math analysis on kinematic models and so on.