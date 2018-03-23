
from mpmath import radians
from sympy import *
from time import time

import math
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0

    # gripper orientation correction as described in lesson
    r = symbols('r')
    p = symbols('p')
    y = symbols('y')
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

    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # the orientation of the end effector would be the roll, pitch yaw combined
    # with the rotation correction
    R_ee = (R_z * R_y * R_x).evalf(subs={r: roll, p: pitch, y: yaw}) * R_corr

    # the wrist center would be offset backward from the end effector
    eePos = Matrix([[px], [py], [pz]])
    wcPos = eePos - 0.303 * R_ee[:, 2]

    # symbols for the DH parameters, q stands for theta, note that q[0] here
    # is actually q1 and d[0] is actually d1 in the lesson notation
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

    ### Inverse Position
    # theta1 can be obtained by projecting the wrist center to the xy plane and
    # calculating the angle between origin-wc vs x-axis
    theta1 = atan2(wcPos[1], wcPos[0])

    # use the triangle diagram to get theta 2 and theta 3
    # side a: distance between link 3 and write center, or sqrt(d[3] ** 2 + a[3] ** 2)
    sa = 1.501
    # side b: distance between link 2 and wrist center, use world coordinates to calculate dist
    dx2Wc = sqrt(wcPos[0] ** 2. + wcPos[1] ** 2.) - 0.35 # offset on xy plane
    dz2Wc = wcPos[2] - 0.75 # offset of z
    sb = sqrt(dx2Wc ** 2. + dz2Wc ** 2.)
    # side c: distance between link 2 and 3, or a[2]
    sc = 1.25

    # use cosine law to get all three angles for theta 2 and 3
    ta = acos((sb ** 2. + sc ** 2. - sa ** 2.) / (2.0 * sb * sc))
    tb = acos((sa ** 2. + sc ** 2. - sb ** 2.) / (2.0 * sa * sc))
    tc = acos((sa ** 2. + sb ** 2. - sc ** 2.) / (2.0 * sa * sb))

    # use the diagram to compute theta 2 and theta 3
    # theta2 would be pi/2 minus angle a, then minus the angle of link2-wc vs X1 axis
    theta2 = pi/2. - ta - atan2(dz2Wc, dx2Wc)
    # theta3 would be the negative of angle b + angle between link3-wc vs X3 (abs(atan2(a[3], d[3]))) minus pi/2
    theta3 = - (tb + 0.036 - pi/2.)

    ### Inverse Orientation
    # homogeneous transformation matrices
    T = []
    for i in range(7):
        T.append(Matrix([[                 cos(q[i]),                -sin(q[i]),              0,                  a[i] ],
                         [ sin(q[i]) * cos(alpha[i]), cos(q[i]) * cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i]) * d[i] ],
                         [ sin(q[i]) * sin(alpha[i]), cos(q[i]) * sin(alpha[i]),  cos(alpha[i]),  cos(alpha[i]) * d[i] ],
                         [                         0,                         0,              0,                     1 ]]))
        T[i] = T[i].subs(CONST_DH)

    # composition of homogeneous transformations
    R0_3 = (T[0] * T[1] * T[2]).evalf(subs={q[0]: theta1, q[1]: theta2, q[2]: theta3})
    R3_6 = R0_3[:3,:3].inv('LU') * R_ee

    # use the R3_6 matrix elements to get theta 4 through 6
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[1,0] ** 2. + R3_6[1,1] ** 2.), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ##
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    thetas = {
        q[0]: theta1,
        q[1]: theta2,
        q[2]: theta3,
        q[3]: theta4,
        q[4]: theta5,
        q[5]: theta6
    }
    T0_EE = T[0].evalf(subs = thetas)
    for i in range(1, 7):
        T0_EE = T0_EE * T[i].evalf(subs = thetas)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = wcPos # <--- Load your calculated WC values in this array
    your_ee = T0_EE[:3, 3] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    for test_case_number in [1, 2, 3]:
        print 'Testing case', test_case_number
        test_code(test_cases[test_case_number])
