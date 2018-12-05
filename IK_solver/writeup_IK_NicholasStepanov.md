## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/DH_table_and_transforms.jpg
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

To write code in IK_server.py I used jupyter notebooks to calculate the angles for test cases and recreated those poses in forward kinematics environment to be approximately at the desired locations/orientations

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

DH parameters and transforms:

![alt text][image1]



#### 2. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

After  deriving wrist center position:
theta1 = arctan(wy/wx)
theta2 and 3 are derived using cosine rule for known sides of the triangle J2, J3, WC
After calculating theta1,2,3 
Create Rotation matrix R3_G for wrist center using R0_3 with substituted theta1,2,3 and Rrpy matrix
theta4 = arctan (R3_G33/ -R3_G13) or (sin(theta4)/cos(theta4))
theta5 = arctan (-R3_G13/cos(theta4), R3_G23) or (sin(theta5)/cos(theta5))
theta6 = arctan (-R3_G22/ R3_G21) or (sin(theta6)/cos(theta6))


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In my Theta 4 and Theta6 calculations I did not account for the possible angles to be -7 rad - 7rad, therefore, my code does not recognize the planning and will occasionally rotate joints 4,6 180 degrees to get into desired position


And just for fun, another example image:
![alt text][image3]


