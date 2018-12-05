# Udacity_Robotics_Projects
Projects I have completed as a part of Udacity RoboND
## RoboND-Rover-Project
In this project I programmed a simulation Rover to autonomously map its environment using openCV and color thresholding. It is also able to detect golden samples with specific color.
## IK_Solver
In this project I have programmed a ROS service which computes the joint angles of a Kuka Arm based on the position and orientation of the end effector. The service then returns these angles and the arm is able to follow given trajectory point by point
## Perception-Project
In this project I have trained a perception pipeline to recognize Point Cluster based on their color and normals histograms. The used objects' information was recorded first and used to train a Support Vector Classifier. This ML method is used inside the Perception pipeline that separates the Point Cloud information received from Robot's sensor from background and noise and divides it into clusters for classification.
## Follow_me
In this project I have trained and created a Fully Convolutional Neural Network for pixel-by-peixel segmentation of an image coming from a simulated environment. The Neural Network recognizes a target in simulation and is then used by a simulated drone to follow the target ignoring other objects that look similar to the target.
## AMCL_Localization
In this project I have created a ROS package that can be used to perform Robot Localization on a given map using Advanced Monte Carlo technique. I have created a Robot Model and used a world and its map provided by Udacity. The robot was able to succesfully localize in the given environment and was able to navigate to the destination point provided by a C++ executable.
## RTAB-Mapping-for-SLAM
In this project I have created a ROS package that can be used to perform mapping of unknown environment by a simulated robot using RTAB-Mapping package. The robot showed a certain degree of accuracy mapping the world around it, but also demonstrated that for the RTAB-Mapping package to work properly it would require the environment to have distinct feature-rich areas.
