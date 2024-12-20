import numpy as np
import matplotlib.pyplot as plt


I = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

zero = np.array([[0], [0], [0]])


def rot_x(tilt): 
    R = np.array([[1,       0,             0     ],
                   [0, np.cos(tilt), -np.sin(tilt)],
                   [0, np.sin(tilt),  np.cos(tilt)]])
    return R

def rot_y(roll): 
    R = np.array([[np.cos(roll),     0,     np.sin(roll)],
                   [       0,         1,       0     ],
                   [ -np.sin(roll),   0,     np.cos(roll)]])
    return R

def rot_z(pan): 
    R = np.array([[np.cos(pan), -np.sin(pan), 0],
                   [np.sin(pan),  np.cos(pan), 0],
                   [     0,            0,      1]])
    return R

def problem_1a():
    R = np.matmul(np.matmul(rot_z(0.50), rot_x(-0.2)), rot_y(1.0))
    print(R)


def problem_1b():
    R_t = np.array([[ 0.2803, -0.6124, -0.7392],
                     [-0.7392,  0.3536, -0.5732],
                     [ 0.6124,  0.7071, -0.3536]])

    # getting the pointing direction
    d = np.matmul(R_t, [0,1,0])
    x,y,z = d[0], d[1], d[2]

    # use inverse kinematics to find pan and tilt
    # then find theta 
    # one solution
    pan = np.arctan2(-x,y)
    tilt = np.arctan2(z, np.sqrt(x**2 + y**2))
    R_x = rot_x(tilt)
    R_z = rot_z(pan)
    R_y = np.matmul(R_x.T, np.matmul(R_z.T, R_t))
    roll = np.arctan2(R_y[0,2], R_y[0,0])
    R = np.matmul(np.matmul(rot_z(pan), rot_x(tilt)), rot_y(roll))
    print("(pan, tilt, roll) =  ({},{},{})".format(pan,tilt,roll))

    # second solution
    pan = np.arctan2(-x,y) - np.pi
    tilt = np.pi - np.arctan2(z, np.sqrt(x**2 + y**2))
    R_x = rot_x(tilt)
    R_z = rot_z(pan)
    R_y = np.matmul(R_x.T, np.matmul(R_z.T, R_t))
    roll = np.arctan2(R_y[0,2], R_y[0,0])
    R = np.matmul(np.matmul(rot_z(pan), rot_x(tilt)), rot_y(roll))
    print("(pan, tilt, roll) =  ({},{},{})".format(pan,tilt,roll))



def problem_2c(theta_1, theta_2, theta_3, theta_4):

    pos = np.array([0,0,0,1])

    bottom_row = np.array([0, 0, 0, 1])
    L = 0.4

    # O -> O'
    T_0 = np.vstack((np.hstack((rot_z(theta_1), zero)), bottom_row))

    # O' -> 1
    p = np.array([[0], [0], [L]])
    T_1 = np.vstack((np.hstack((I, p)), bottom_row))

    # 1 -> 1'
    T_2 = np.vstack((np.hstack((rot_y(theta_2), zero)), bottom_row))

    # 1' -> 2
    p = np.array([[0], [L], [0]])
    T_3 = np.vstack((np.hstack((I, p)), bottom_row))

    # 2 -> 2'
    T_4 = np.vstack((np.hstack((rot_z(-theta_3), zero)), bottom_row))

    # 2' -> 3
    p = np.array([[0], [0], [-L]])
    T_5 = np.vstack((np.hstack((I, p)), bottom_row))

    # 3 -> 3'
    T_6 = np.vstack((np.hstack((rot_y(theta_4), zero)), bottom_row))

    # 3' -> 4
    p = np.array([[0], [L], [0]])
    T_7 = np.vstack((np.hstack((I, p)), bottom_row))

    # 4 -> t
    T_8 = np.vstack((np.hstack((I, zero)), bottom_row))

    T = T_0 @ T_1 @ T_2 @ T_3 @ T_4 @ T_5 @ T_6 @ T_7 @ T_8
    print(T)

    tip_pos = T @ pos
    print("Print Tip pos: {}".format(tip_pos))

    x_axis = np.array([T[0,0], T[1,0], T[2,0]])
    #print("i hat = {}".format(x_axis))
    angle_above_xy = np.arctan2(x_axis[2], np.sqrt(x_axis[0]**2 + x_axis[1]**2))
    print("Angle above xy plane: {} rads = {} deg".format(angle_above_xy, angle_above_xy * 180/np.pi))
    


if __name__ == "__main__":
    #problem_1a()
    #problem_1b()
    problem_2c(np.pi/4, -np.pi/3, np.pi/4, np.pi/6)