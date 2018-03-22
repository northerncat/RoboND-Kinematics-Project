#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
    #
    #
    # Create Modified DH parameters
    #
    #
    # Define Modified DH Transformation matrix
    #
    #
    # Create individual transformation matrices
    #
    #
    # Extract rotation matrices from the transformation matrices
    #
    #
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        #
        #
        # Calculate joint angles using Geometric IK method
        #
        #

        r = symbols('r')
        pitch = symbols('pitch')
        yaw = symbols('yaw')
        R_x = Matrix([[ 1,       0,        0 ],
                      [ 0,  cos(r),  -sin(r) ],
                      [ 0,  sin(r),   cos(r) ]])
        R_y = Matrix([[  cos(p),  0,  sin(p) ],
                      [       0,  1,       0 ],
                      [ -sin(p),  0,  cos(p) ]])
        R_z = Matrix([[ cos(y),  -sin(y), 0 ],
                      [ sin(y),   cos(y), 0 ],
                      [      0,        0, 1 ]])
        R_corr = R_z.evalf(subs={y: pi}) * R_y.evalf(subs={p: -pi/2})

        R_ee = (R_z * R_y * R_x).evalf(subs={r: roll, p: pitch, y: yaw}) * R_corr

        eePos = Matrix([[px], [py], [pz]])
        wcPos = eePos - 0.303 * R_ee[:, 2]

        # symbols for the DH parameters, q stands for theta
        q = symbols('q1:8')
        d = symbols('d1:8')
        a = symbols('a0:7')
        alpha = symbols('alpha0:7')

        # constant DH parameters
        CONST_DH = {
            alpha[0]:     0,  a[0]:      0,  d[0]: 0.75,
            alpha[1]: -pi/2.,  a[1]:   0.35,  d[1]: 0.0,   q[1]: q[1] - pi/2.,
            alpha[2]:     0,  a[2]:   1.25,  d[2]: 0.00,
            alpha[3]: -pi/2.,  a[3]: -0.054,  d[3]: 1.5,
            alpha[4]:  pi/2.,  a[4]:      0,  d[4]: 0.0,
            alpha[5]: -pi/2.,  a[5]:      0,  d[5]: 0.0,
            alpha[6]:     0,  a[6]:      0,  d[6]: 0.303,         q[6]: 0.0
        }

        theta1 = atan2(wcPos[1], wcPos[0])

        # side a: distance between link 3 and write center, (sqrt(d[3] ** 2 + a[3] ** 2)).evalf(subs=CONST_DH)
        sa = 1.501
        # side b: distance between link 2 and wrist center, use world coordinates to calculate dist
        dx2Wc = sqrt(wcPos[0] ** 2. + wcPos[1] ** 2.) - 0.35
        dz2Wc = wcPos[2] - 0.75
        sb = sqrt(dx2Wc ** 2. + dz2Wc ** 2.)
        # side c: distance between link 2 and 3, a[2]
        sc = 1.25

        # use cosine law to get all three angles for theta 2 and 3
        ta = acos((sb ** 2. + sc ** 2. - sa ** 2.) / (2.0 * sb * sc))
        tb = acos((sa ** 2. + sc ** 2. - sb ** 2.) / (2.0 * sa * sc))
        tc = acos((sa ** 2. + sb ** 2. - sc ** 2.) / (2.0 * sa * sb))

        # use the diagram to compute theta2 and theta 3
        # theta2 would be pi/2 minus angle a, then minus the angle of link2 to wrist center vs X1 axis
        theta2 = pi/2. - ta - atan2(dz2Wc, dx2Wc)
        # theta3 would be the negative of angle b + angle between link 3 to wrist center (abs(atan2(a[3], d[3]))) minus pi/2
        theta3 = - (tb + 0.036 - pi/2.)

        # homogeneous transformation matrices
        T = []
        for i in range(7):
            T.append(Matrix([[                 cos(q[i]),                -sin(q[i]),              0,                  a[i] ],
                             [ sin(q[i]) * cos(alpha[i]), cos(q[i]) * cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i]) * d[i] ],
                             [ sin(q[i]) * sin(alpha[i]), cos(q[i]) * sin(alpha[i]),  cos(alpha[i]),  cos(alpha[i]) * d[i] ],
                             [                         0,                         0,              0,                     1 ]]))
            T[i] = T[i].subs(CONST_DH)

        # composition of homogeneous transformations
        R0_3 = (T[0][:3,:3] * T[1][:3,:3] * T[2][:3,:3]).evalf(subs={q[0]: theta1, q[1]: theta2, q[2]: theta3})
        R3_6 = R0_3.inv('LU') * R_ee

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2] ** 2. + R3_6[2,2] ** 2.), R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            ###

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
