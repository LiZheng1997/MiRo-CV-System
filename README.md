# MiRo-CV-System

This is my first version of an bio-inspired visual system which is based on OpenCV APIs and ANN, and the final aim of this project is intercepting a moving target. I pre-defined **three targets**, named pedestrians, MiRo robots and MiRo toy balls. In total, there are three scenarios for different deploying situations, like on-board;  off-board and in a simulator (Gazebo). Some codes are inspired from **code samples** in the MDK-2019 version. Thanks for their brilliant work on building MiRo robots. I am also one of the big fan loving biomimetic robots. Link: http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Introduction

***Branches Notes:*** 

The master branch is the official released version, and the core version is the core part of this system, also the dev branch contains all developed codes.

[TOC]

# News

**2023-10** MDK-20230105 is supported now!

**2023-09** ROS Noetic is supported now!



# Requirements

## Software

1. Ubuntu20.04
2. ROS Noetic
3. MiRo MDK 20230105
4. OpenCV > 3.4
5. Python

## Hardware

1. a MiRo robot
2. a Laptop with GPU card is better



# Contribution

Any contributions are welcomed, if you love playing with MiRo robots, you can use this repo as a good start of applying computer vision algorithms on a [Biomimetic ](https://en.wikipedia.org/wiki/Biomimetics) robot.



## 1. Background

First of all, this project is my Master's dissertation project in 2019, now I uploaded all codes and more improvements will be implemented in the future. All relevant resources will be uploaded, including training sample images and model files. You can find some model files in the root path of this repo, a fold named ***models***. Training sample images can be found here: [Google Drive](https://drive.google.com/drive/folders/1owF3loF_p_iO_xP7X3yBjYI6e0c9NLm3?usp=sharing)



## 2. Methods

This CV system is mainly based on **OpenCV library**, which uses some conventional computer vision algorithms, like **Hough circle detection**, **Gaussian filter**, **Median filter**, **Optical flow estimation** and **stereo camera depth calculation**, etc. Also, **Deep Neural Network models** are used in this project to achieve real time detection.  Apart from perception algorithms, a **bio-inspired method** is also designed in this system using a end-to-end paradigm, it refers to the famous **[Braitenberg vehicle](https://en.wikipedia.org/wiki/Braitenberg_vehicle) theory**. Path planning module and Localization module are not added into this system yet now.

### 2.1 Braitenberg vehicles

In this system, I implemented two kinds of braitenberg vehicles, **2b (aggression) vehicle** and **3a (love) vehicle**,   you can find these two types in following two pictures.

<img src="assets\miro\image-20240804165037267.png" alt="2b" style="zoom:67%;" />



![3a](assets\miro\image-20240804165358095.png)

### 2.2 The Kinematics of interception task

The first picture shows the kinematic model about that a miro robot wants to intercept another miro robot.

<img src="assets\miro\image-20240804171420804.png" alt="kinematic model" style="zoom: 67%;" />

The second picture shows a similar interception geometry explanation.

<img src="assets\miro\image-20240804172129418.png" alt="image-20240804172129418" style="zoom: 67%;" />





## 3. Framework

### 3.1 Detector Module

#### 3.1.1. MiRo Detection Node

This miro robot detection node is based on Cascade Classifier, the classifier model is trained using collected indoor MiRo robots' appearance data. A positive sample is a MiRo robot in the center of an image , a negative sample is some other indoor objects in a lab. Hence, this cascade classifier needs to predict whether there is a MiRo robot in one frame.

#### 3.1.2. Ball Detection Node

This node is based on the **Hough Circle** detection in the OpenCV lib. And this node might  better use in a Gazebo simulator for showing a better effect. The reason is that the real world environment is sometimes too noisy for this detection algorithm to work well, the effect of detection is unstable.

#### 3.1.3. Pedestrian Detection Node

This node is based on a DNN module in the OpenCV lib or independent models. Using some traditional CNN model, like darknet, faster-rcnn and yolov3, to achieve the functionality of pedestrian detection in real world with a considerable latency.

### 3.2 Tracker Module

#### 3.2.1. Single-Tracking Node

This node is based on the tracker APIs in OpenCV . There are 8 trackers in total after opencv3.4 version. So, if you want to use this project, you have to use a higher or equal version than opecnv3.4, or some unexpected errors might encounter. The TLD tracker is the one I prefer to use in indoor environment with occlusion situation.

#### 3.2.2. Multi-Tracking Node

This node is based on the multitracker API in OpenCV.

### 3.3 Control Module

The Control module contains a direction keeper node, this is the main function for maintaining the yaw angle of a MiRo robot.

#### 3.3.1. Orientation Control Node

This node is based on a designed **bio-inspired** orientation correction algorithms. Orientation errors are calculated based on the yaw angle variations between the view line and the ego-to-target line. Hence, the MiRo robot will be driven to point its head towards the target and keep following the target, then it can intercept this moving target with considerable higher linear velocity. Moreover, the angular velocity will be produced based on the angle variation mentioned before.

#### 3.3.2. Safety Controller Node

This node is based on a sonar sensor on the MiRo robot's nose. I use the raw data from the sonar sensor to estimate the distance between the target and the MiRo robot. This is not accurate enough in real time tests.

### 3.4. Path Planning Module

#### 3.4.1. Path planning Node

Global path planning and Local path planning are designed for generating paths of intercepting another moving target. A kinematic model is necessary for this node to work smoothly in real time. But, in this version, I did not add path planning module to produce real time planning paths. Instead, I merely use aforementioned Orientation Control module to implement a bio-inspired way of approaching a moving target without applying a planning method.

### 3.5 Localization Module

#### 3.5.1. Kalman Filter Pose Estimation Node

Using last second pose and utilize Kalman Filter to predict the current ego pose. Still, this node is not added to the first version, because the objective of using a pose estimation node is utilized in the path planning module.

#### 3.5.2. Wheel Odometry Node

This wheel odometry node is mainly designed for localizing the MiRo robot's global position, but accumulated errors should be considered and drift issue might need be solved.

#### 3.5.3. Visual Odometry Node

This visual odometry node is used for improving the localization accuracy, and it will be fused with wheel odometry to generate a more accurate global position.



## 4. Summary

All codes are implemented based on doing self-learning roughly in 5 months. Necessary knowledge includes ROS, Computer Vision basic theories, OpenCV APIs, Physical Robots' experiments, Linux system, Machine Learning, Math analysis on kinematic models and so on.



## TODO

- [x] Simplify codes to a core version
- [x] Update the compatible ROS version to ROS1 Noetic version
- [x] Update the MDK version to MDK202301
- [ ] Add and test the Wheel Odometry node
- [ ] Add and test the Visual Odometry node
- [ ] Add the path planning module into the system for comparation
- [ ] update previous bio-inspired and RL methods



## More about MiRo Robot

If you want to know more about this biomimetic MiRo robot, please click this [link](https://www.miro-e.com/robot) to find more on their official websites.



## References

[1] Belkhouche, F. and Belkhouche, B., 2004, December. On the tracking and interception of a moving object by a wheeled mobile robot. In *IEEE Conference on Robotics, Automation and Mechatronics, 2004.* (Vol. 1, pp. 130-135). IEEE.

[2] Headleand, C.J. and Teahan, W., 2016, July. Towards ethical robots: Revisiting Braitenberg's vehicles. In *2016 SAI Computing Conference (SAI)* (pp. 469-477). IEEE.

[3] Application of Computer Vision for A Biomimetic Robot

