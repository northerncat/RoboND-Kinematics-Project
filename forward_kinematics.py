#!/usr/bin/env python

import tf
import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# symbols for the DH parameters, q stands for theta
q = symbols('q1:8')
d = symbols('d1:8')
a = symbols('a0:7')
alpha = symbols('alpha0:7')

# constant DH parameters
s = {
	alpha[0]:     0,  a[0]:      0,  d[0]: 0.75,
	alpha[1]: -pi/2,  a[1]:   0.35,  d[1]: 0.0,   q[1]: q[1] - pi/2,
	alpha[2]:     0,  a[2]:   1.25,  d[2]: 0.00,
	alpha[3]: -pi/2,  a[3]: -0.054,  d[3]: 1.5,
	alpha[4]:  pi/2,  a[4]:      0,  d[4]: 0.0,
	alpha[5]: -pi/2,  a[5]:      0,  d[5]: 0.0,
	alpha[6]:     0,  a[6]:      0,  d[6]: 0.303,         q[6]: 0.0
}

# homogeneous transformation matrices
T = []
for i in range(7):
	T.append(Matrix([[                 cos(q[i]),                -sin(q[i]),              0,                  a[i] ],
		             [ sin(q[i]) * cos(alpha[i]), cos(q[i]) * cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i]) * d[i] ],
		             [ sin(q[i]) * sin(alpha[i]), cos(q[i]) * sin(alpha[i]),  cos(alpha[i]),  cos(alpha[i]) * d[i] ],
		             [                         0,                         0,              0,                     1 ]]))
	T[i] = T[i].subs(s)

# composition of homogeneous transformations
T_base_to_gripper = T[0]
for i in range(1, 7):
	T_base_to_gripper = simplify(T_base_to_gripper * T[i])

# gripper orientation correction as described in lesson
roll = symbols('roll')
pitch = symbols('pitch')
yaw = symbols('yaw')
R_x = Matrix([[ 1,          0,           0, 0 ],
	          [ 0,  cos(roll),  -sin(roll), 0 ],
	          [ 0,  sin(roll),   cos(roll), 0 ],
	          [ 0,          0,           0, 1 ]])
R_y = Matrix([[  cos(pitch),  0,  sin(pitch), 0 ],
	          [           0,  1,           0, 0 ],
	          [ -sin(pitch),  0,  cos(pitch), 0 ],
	          [           0,  0,           0, 1 ]])
R_z = Matrix([[ cos(yaw),  -sin(yaw), 0, 0 ],
	          [ sin(yaw),   cos(yaw), 0, 0 ],
	          [        0,          0, 1, 0 ],
	          [        0,          0, 0, 1 ]])
R_corr = simplify(R_z.subs({yaw: pi}) * R_y.subs({pitch: -pi/2}))

# the final transformation matrix
# T_base_to_gripper = simplify(T_base_to_gripper * R_corr)
# print T_base_to_gripper.evalf(subs={
# 	    q[0]: 0.0,
# 	    q[1]: 0.0,
# 	    q[2]: 0.0,
# 	    q[3]: 0.0,
# 	    q[4]: 1.57,
# 	    q[5]: 0.0
# 	})

(r, p, y) = tf.transformations.euler_from_quaternion(
    [0, 0.70592, 0, 0.70592])
R_ee = simplify(R_z * R_y * R_x).subs({ roll: r, pitch: p, yaw: y})
print R_ee * R_corr