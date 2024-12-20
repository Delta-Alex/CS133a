import numpy as np

def quiz2_3():
    e_z = np.array([0,0,1])
    cross1 = np.cross(e_z, np.array([2,2,0])) #For J1
    cross2 = np.cross(e_z, np.array([2,1,0])) #For J2
    cross3 = np.cross(e_z, np.array([1,1,0])) #For J3
    cross4 = np.cross(e_z, np.array([1,0,0])) #For J4

    print(cross1)
    print(cross2)
    print(cross3)
    print(cross4)

def quiz2_3b():
    J = np.array([[-2, -1, -1, 0],
                  [2, 2, 1, 1],
                  [1, 1, 1, 1]])
    xr_dot = np.array([[0], 
                       [-1],
                       [-1]])
    qdot = np.linalg.pinv(J) @ xr_dot
    print(qdot)


def quiz2_3e():
    J = np.array([[-2,-1],
                  [2, 1],
                  [1, 1]])
    xr_dot = np.array([[0], 
                       [-1],
                       [-1]])
    qdot = np.linalg.pinv(J) @ xr_dot
    print(qdot)

def problem1a():
    e_z = np.array([0,0,1])
    cross2 = np.cross(e_z, np.array([2,1,0])) #For J2
    cross3 = np.cross(e_z, np.array([1,0,0])) #For J3
    cross4 = np.cross(e_z, np.array([0,1,0])) #For J4

    print(cross2)
    print(cross3)
    print(cross4)

def problem1c():
    J = np.array([[1, -1, 0, -1],
                  [0, 2, 1, 0]])
    xr_dot = np.array([[-1], 
                       [1]])
    W = np.diag([1, 1/2, 1/3, 1/4])
    W_inv = np.linalg.inv(W)
    J_T = np.transpose(J)
    A = J @ W_inv @ W_inv @ J_T
    qdot = W_inv @ W_inv @ J_T @ np.linalg.inv(A) @ xr_dot
    print(qdot)

def problem1d():
    J = np.array([[1, 0, -1],
                  [0, 1, 0]])
    xr_dot = np.array([[-1], 
                       [1]])
    W = np.diag([1, 1/3, 1/4])
    W_inv = np.linalg.inv(W)
    J_T = np.transpose(J)
    A = J @ W_inv @ W_inv @ J_T
    qdot = W_inv @ W_inv @ J_T @ np.linalg.inv(A) @ xr_dot
    qdot = np.array([[qdot[0,0]],
                     [0],
                     [qdot[1,0]],
                     [qdot[2,0]]])
    print(qdot)

def problem1e():
    J = np.array([[1, -1],
                  [0, 0]])
    xr_dot = np.array([[-1], 
                       [1]])
    W = np.diag([1, 1/4])
    W_inv = np.linalg.inv(W)
    J_bar = J @ W_inv
    qdot = W_inv @ np.linalg.pinv(J_bar) @ xr_dot
    
    qdot = np.array([[qdot[0,0]],
                     [0],
                     [0],
                     [qdot[1,0]]])
    print(qdot)



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

def problem2a():
    q = np.array([[np.radians(0)],
                  [np.radians(44.5)],
                  [np.radians(90)]])
    J = Jac(q)
    print("J: {}".format(J))
    u, s, vT = np.linalg.svd(J)
    print("U: {}".format(u))
    print("S: {}".format(np.diag(s)))
    print("V^T: {}".format(vT))

def problem2b():
    q = np.array([[np.radians(0)],
                  [np.radians(44.5)],
                  [np.radians(90)]])
    xr_dot = np.array([[1],
                  [0],
                  [1]])
    J = Jac(q)
    qdot = np.linalg.inv(J) @ xr_dot
    print("qdot : {}".format(qdot))
    xdot = J @ qdot
    print("xdot : {}".format(xdot))

    #u, s, vT = np.linalg.svd(J)
    #s = np.diag(s)
    #qdot = np.transpose(vT) @ np.linalg.inv(s) @ np.transpose(u) @ xr_dot
    #xdot = J @ qdot
    #print("xdot : {}".format(xdot))

def problem2cd(gamma):
    q = np.array([[np.radians(0)],
                  [np.radians(44.5)],
                  [np.radians(90)]])
    xr_dot = np.array([[1],
                  [0],
                  [1]])
    J = Jac(q)
    JT = np.transpose(J)
    A = J @ JT + gamma**2 * np.eye(3)
    JW_inv = JT @ np.linalg.inv(A)
    qdot = JW_inv @ xr_dot
    print("qdot : {}".format(qdot))
    xdot = J @ qdot
    print("xdot : {}".format(xdot))


    #u, s, vT = np.linalg.svd(J)
    #sb = np.square(s) + gamma**2
    #s = np.diag(s/sb)
    #qdot = np.transpose(vT) @ s @ np.transpose(u) @ xr_dot
    #xdot = J @ qdot
    #print("xdot : {}".format(xdot))

def problem2e(gamma):
    q = np.array([[np.radians(0)],
                  [np.radians(44.5)],
                  [np.radians(90)]])
    xr_dot = np.array([[1],
                  [0],
                  [1]])
    J = Jac(q)

    u, s, vT = np.linalg.svd(J)
    uT = np.transpose(u)
    v = np.transpose(vT)
    diagonals = []
    for si in s:
        if np.abs(si) >= gamma:
            diagonals.append(1/si)
        else:
            diagonals.append(si/(gamma**2))
    diagonals = np.array(diagonals)
    S = np.diag(diagonals)
    qdot = v @ S @ uT @ xr_dot
    print("qdot : {}".format(qdot))
    xdot = J @ qdot
    print("xdot : {}".format(xdot))

    print("squared error: {}".format(np.linalg.norm(xr_dot - xdot) **2))
    


if __name__ == "__main__":
    #problem1a()
    #problem1c()
    #problem1d()
    #problem1e()
    #problem2a()
    #problem2b()
    #problem2cd(0.01)
    #problem2cd(0.1)
    #problem2e(0.1)
    quiz2_3e()