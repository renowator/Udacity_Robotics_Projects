#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *



def get_wrist_center(x,y,z,a,b,g): #gamma - roll beta -yaw alpha - pitch
    a1 = cos(a) * cos(b)
    d1 = sin(a)*cos(b)*1.0
    g1 = -sin(b)
    d7 = 0.303
    WC_x =   x - (d7 * a1 * 1.0) #in the world coordinates it is changing along x-direction..DH conventions Z-direction
    WC_y =   y - (d7 * d1 * 1.0)
    WC_z =   z - (d7 * g1 * 1.0)
    return WC_x,WC_y,WC_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
	    rtd = 180/mp.pi
	    dtr = mp.pi/180
            print "Define DH param symbols"
            # Define DH param symbols
	    calpha0, calpha1, calpha2, calpha3, calpha4, calpha5, calpha6= symbols('calpha0:7')
	    salpha0, salpha1, salpha2, salpha3, salpha4, salpha5, salpha6= symbols('salpha0:7')
	    a0, a1, a2, a3, a4, a5, a6= symbols('a0:7')
	    d1, d2, d3, d4, d5, d6, d7= symbols('d1:8')
	    

            
            # Joint angle symbols
	    q1, q2, q3, q4, q5, q6, q7= symbols('q1:8')

      
            # Modified DH params
	    s = {calpha0: 1,salpha0:0, a0: 0, d1: 0.75,
		 calpha1: 0,salpha1: -1, a1: 0.35, d2: 0, q2:q2 - mp.pi/2,
		 calpha2: 1,salpha2: 0, a2: 1.25, d3: 0,
		 calpha3: 0,salpha3: -1, a3: -0.054, d4: 1.5,
		 calpha4: 0,salpha4: 1, a4: 0, d5: 0,
		 calpha5: 0,salpha5: -1, a5: 0, d6: 0,
		 calpha6: 1 ,salpha6: 0 ,  a6: 0, d7: 0.303, q7: 0}
	    d_1 = s[d1]
	    a_1 = s[a1]
	    a_2 = s[a2]
	    a_3 = s[a3]
	    d_4 = s[d4]
	    d_7 = s[d7]
	    joint3_side = (a_3**2 +d_4**2)**0.5
	    joint3_angle = atan2(-a_3, d_4)
            print "Define Matrices"
            # Define Modified DH Transformation matrix
 	    T0_1 = Matrix([[           cos(q1),            -sin(q1),           0,             a0],
			   [sin(q1)*calpha0, cos(q1)*calpha0, -salpha0, -salpha0*d1],
			   [sin(q1)*salpha0, cos(q1)*salpha0,  calpha0,  calpha0*d1],
			   [                0,                 0,           0,             1]])
	    T0_1 = T0_1.subs(s)
	    T1_2 = Matrix([[           cos(q2),            -sin(q2),           0,             a1],
			   [sin(q2)*calpha1, cos(q2)*calpha1, -salpha1, -salpha1*d2],
			   [sin(q2)*salpha1, cos(q2)*salpha1,  calpha1,  calpha1*d2],
			   [                0,                 0,           0,             1]])
	    T1_2 = T1_2.subs(s)
	    T2_3 = Matrix([[           cos(q3),            -sin(q3),           0,             a2],
			   [sin(q3)*calpha2, cos(q3)*calpha2, -salpha2, -salpha2*d3],
			   [sin(q3)*salpha2, cos(q3)*salpha2,  calpha2,  calpha2*d3],
			   [                0,                 0,           0,             1]])
	    T2_3 = T2_3.subs(s)
	    T3_4 = Matrix([[           cos(q4),            -sin(q4),           0,             a3],
			   [sin(q4)*calpha3, cos(q4)*calpha3, -salpha3, -salpha3*d4],
			   [sin(q4)*salpha3, cos(q4)*salpha3,  calpha3,  calpha3*d4],
			   [                0,                 0,           0,             1]])
	    T3_4 = T3_4.subs(s)
	    T4_5 = Matrix([[           cos(q5),            -sin(q5),           0,             a4],
			   [sin(q5)*calpha4, cos(q5)*calpha4, -salpha4, -salpha4*d5],
			   [sin(q5)*salpha4, cos(q5)*salpha4,  calpha4,  calpha4*d5],
			   [                0,                 0,           0,             1]])
	    T4_5 = T4_5.subs(s)
	    T5_6 = Matrix([[           cos(q6),            -sin(q6),           0,             a5],
			   [sin(q6)*calpha5, cos(q6)*calpha5, -salpha5, -salpha5*d6],
			   [sin(q6)*salpha5, cos(q6)*salpha5,  calpha5,  calpha5*d6],
			   [                0,                 0,           0,             1]])
	    T5_6 = T5_6.subs(s)
	    T6_7 = Matrix([[           cos(q7),            -sin(q7),           0,             a6],
			   [sin(q7)*calpha6, cos(q7)*calpha6, -salpha6, -salpha6*d7],
			   [sin(q7)*salpha6, cos(q7)*salpha6,  calpha6,  calpha6*d7],
			   [                0,                 0,           0,             1]])
	    T6_7 = T6_7.subs(s)


            # Create individual transformation matrices
            T0_2 = T0_1 * T1_2
            T0_3 = T0_2 * T2_3
            T0_4 = T0_3 * T3_4
            T0_5 = T0_4 * T4_5
            T0_6 = T0_5 * T5_6
            T0_7 = T0_6 * T6_7

            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            print "Calculate IK"

            # Calculate joint angles using Geometric IK method
	    # Get wrist center position
            wx, wy, wz = get_wrist_center(px, py, pz, pitch, yaw, roll)
            #print "wrist:", str(wx), str(wy), str(wz)
            
            R_z = Matrix([[cos(mp.pi), -sin(mp.pi), 0, 0],
		          [sin(mp.pi),  cos(mp.pi), 0, 0],
		          [0,0,1,0],
		          [0,0,0,1]])
	    R_y = Matrix([[cos(-mp.pi/2), 0, sin(-mp.pi/2), 0],
		          [0,1,0,0],
		          [-sin(-mp.pi/2), 0, cos(-mp.pi/2), 0],
		          [0,0,0,1]])
		   
	    R_corr =  R_z * R_y
	    #print T_total
            
	    #Theta 1 is inverse tangent of wy and wx

	    theta1 = atan2(wy, wx)
           
	    #Solve for J2, J3, WC triangle
           
	    l_on_plane = (wx**2 + wy**2)**0.5
	    side = ((l_on_plane - 0.35)**2 + (wz-0.75)**2)**0.5
	    #print "distance between joint 2 and wrist:",side
            C = acos((1.25**2 + joint3_side**2 - side**2) / (2 * 1.25 * joint3_side))
            B = acos((1.25**2 + side**2 - joint3_side**2) / (2 * 1.25 * side))
 

	    #Theta3 is 90 degrees - angle at J3
	    theta3 = mp.pi/2 - joint3_angle - C
            
	    #Theta 2 is the difference between 90 degree triangle with Theta 2 = 0, J2 and WC and angle 
            #at J2 in the triangle

	    angle_q2_0 = atan2(wz - 0.75, l_on_plane - 0.35)
	    theta2 =mp.pi/2 - angle_q2_0 - B
	    #print "theta1", float(theta1)
	    #print "theta2" , float(theta2)
            #print "theta 3:" , float(theta3)
	    if (x <= len(req.poses) ):
            #Theta 4,5,6 from Rotation Matrice values
                Rrpy = Matrix([[    cos(yaw)*cos(pitch),   cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),    cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
                           [    sin(yaw)*cos(pitch),   sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),    sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
                           [            -sin(pitch),             cos(pitch)*sin(roll),                                       cos(pitch)*cos(roll)               ]])
	        R0_3 = T0_1*T1_2*T2_3
	        R0_3 = R0_3.subs({q1:theta1, q2:theta2, q3:theta3})
	        R0_3 = R0_3[:3,:3]
	        R_corr = R_corr[:3,:3]
	        Rrpy = Rrpy * R_corr
 	        R3_G = R0_3.T * Rrpy  
     	        #R3_G_s = T3_4*T4_5*T5_6*T6_7
	        #R3_G_s = R3_G_s[:3,:3]
	    	#print "calculated Rotation Matrice:", R3_G,"\n", R3_G_s
		#print -R3_G[2,2], R3_G_s[2,2], R3_G[0,2], R3_G_s[0,2]
	    	r13 = -R3_G[0,2]
	        theta4 = atan( R3_G[2,2]/ r13)
                #print "theta 4 ", float(theta4)
	        #print R3_G[0,2]/cos(theta4), R3_G_s[0,2]/cos(q4), R3_G[1,2], R3_G_s[1,2]

	        theta5 = atan2(r13/cos(theta4), R3_G[1,2])
	        
	        #print "theta 5 ", float(theta5)
		#print -R3_G[1,1],R3_G_s[1,1], R3_G[1,0], R3_G_s[1,0]
	        r22 = -R3_G[1,1]
		theta6 = atan(r22/ R3_G[1,0])
		#print "theta6 ", float(theta6)
	        #print "yaw ", yaw*rtd

	   	#print "pitch ", pitch*rtd
	        
	        #print "roll " , roll*rtd
	        
	    else:
		theta4 = theta5 = theta6 = 0.0
		
	  
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
