'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should simply use NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

'''

import numpy as np


#
#  EXAMPLE CODE
#
#  TO SEE HOW TO USE NUMPY, LOOK AT NumPyExamples.py!
#
# REMOVE/COMMENT THIS OUT TO AVOID ALL THE EXTRA PRINTS
#import hw5code.NumPyExamples

def example():
    print("EXAMPLE CODE:")
    
    # RANDOM SAMPLE CODE 3x1 column vec, 3x3 matrix:
    q    = np.array([0.1, 0.3, -0.5]).reshape(-1,1)
    xdot = np.array([3,   6,    2  ]).reshape(-1,1)
    J    = Jac(q)
    qdot = np.linalg.inv(J) @ xdot
    print("qdot:\n", qdot)

    print("sin(q):\n",      np.sin(q))
    print("sin(q[0,0]):\n", np.sin(q[0,0]))
    print("sin(q[1,0]):\n", np.sin(q[1,0]))
    print("sin(q[2,0]):\n", np.sin(q[2,0]))
    # Remember every vector is a 2D object and needs 2 indices!


#
#  Forward Kinematics
#
def fkin(q):
    #print("Put the forward kinematics here")
    theta_pan, theta_1, theta_2 = q[0,0], q[1,0],  q[2,0]
    x_tip = -np.sin(theta_pan) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    y_tip = np.cos(theta_pan) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    z_tip = np.sin(theta_1) + np.sin(theta_1 + theta_2)
    x = np.array([x_tip, y_tip, z_tip]).reshape(-1,1)

    # Return the tip position as a numpy 3x1 column vector.
    return x


#
#  Jacobian
#
def Jac(q):
    theta_pan, theta_1, theta_2 = q[0,0], q[1,0],  q[2,0]
    sum_cos = np.cos(theta_1) + np.cos(theta_1 + theta_2)
    sum_sin = np.sin(theta_1) + np.sin(theta_1 + theta_2)
    J = np.eye(3)

    # first row
    theta_12 = theta_1 + theta_2
    J[0] = np.array([-np.cos(theta_pan) * sum_cos, 
                     np.sin(theta_pan) * sum_sin, 
                     np.sin(theta_pan) * np.sin(theta_12)])
    # second row
    J[1] = np.array([-np.sin(theta_pan) * sum_cos, 
                     -np.cos(theta_pan) * sum_sin, 
                     -np.cos(theta_pan) * np.sin(theta_12)])

    # third row
    J[2] = np.array([0, 
                     np.cos(theta_1) + np.cos(theta_12), 
                     np.cos(theta_12)])

    # Return the Jacobian as a numpy 3x3 matrix.
    return J


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Run the example code.  FEEL FREE TO REMOVE.
    example()

    # First (given) test case with following joint coordinates.  Make
    # q a column vector by writing it as a list of lists.
    print("TEST CASE #1:")
    q = np.array([[np.radians(20)],
                  [np.radians(40)],
                  [np.radians(-30)]])
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

    # Second test case with following joint coordinates.  Make
    # q a column vector by explicitly reshaping.
    print("TEST CASE #2")
    q = np.radians(np.array([30, 30, 60])).reshape(3,1)
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

if __name__ == "__main__":
    main()
