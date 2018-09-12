#! /usr/bin/env python
import numpy as np


A = np.matrix([[1, 2, 3, 4], [5, 6, 7, 8]])  # Define a 2x4 matrix
b = np.zeros([4, 1])  # Define a 4 vector (ie a 4x1 matrix) initialized with 0
c = A * b             # Obtain c by multiplying A by b.


from pinocchio.utils import *

eye(6)                      # Return a 6x6 identity matrix
zero(6)                     # Return a zero 6x1 vector
zero([6, 4])                # Return az zero 6x4 matrix 
rand(6)              # Random 6x1 vector
isapprox(zero(6), rand(6))  # Test epsilon equality
#mprint(rand([6, 6]))        # Matlab-style print
skew(rand(3))               # Skew "cross-product" 3x3 matrix from a 3x1 vector
cross(rand(3), rand(3))     # Cross product of R^3
rotate('x', 0.4)            # Build a rotation matrix of 0.4rad around X.	

import pinocchio as se3

R = eye(3); p = zero(3)
M0 = se3.SE3(R, p)
M = se3.SE3.Random()
M.translation = p; M.rotation = R
v = zero(3); w = zero(3)
nu0 = se3.Motion(v, w)
nu = se3.Motion.Random()
nu.linear = v; nu.angular = w
f = zero(3); tau = zero(3)
phi0 = se3.Force(f, tau)
phi = se3.Force.Random()
phi.linear = f; phi.angular = tau

from pinocchio.robot_wrapper import RobotWrapper
from os.path import join

PKG = '/home/alumno04/dev_samir/ros/sawyer_ws/src/'
URDF = join(PKG, '/sawyer_robot/sawyer_description/urdf/model1.urdf')
robot = RobotWrapper(URDF, [PKG])