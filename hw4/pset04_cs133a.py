import numpy as np
import matplotlib.pyplot as plt



def problem_1a(theta_0, w_0, theta_f, w_f, t0, tf):
    """
    Performs cubic spline motion on 
    1 DOF robot. Takes in initial values of
    the motion

    Args:
        theta_0 : initial angular position (radians)
        w_0 : initial angular velocity
        t0 : inital time t
        tf : final time t
    """
    T = tf - t0
    A = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [1,T, T**2, T**3],
                  [0, 1, 2*T, 3*(T**2)]])
    if (np.linalg.det(A) == 0):
        print("Matrix is not invertible")
        return -1
    values = np.array([[theta_0], [w_0], [theta_f], [w_f]])
    consts = np.matmul(np.linalg.inv(A), values)
    print(consts)
    a,b,c,d = consts[0,0], consts[1,0], consts[2,0], consts[3,0]

    N = 1000 #number of points to plot
    thetas = []
    omegas = []
    times = []
    for i in range(N+1):
        t = t0 + (T/N) * i
        theta = a + b*t + c*(t**2) + d*(t**3)
        omega = b + 2*c*t + 3*d*(t**2)

        times.append(t)
        thetas.append(theta)
        omegas.append(omega)

    print("Max position reached = {} radians".format(max(thetas)))
    plt.plot(times, thetas)
    plt.title("Cubic Spline Motion")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (radians)")
    plt.show()

    plt.plot(times, omegas)
    plt.title("Cubic Spline Motion")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rads/s)")
    plt.show()




def problem_1b(theta_0, w_0, a_0, theta_f, w_f, a_f, t0, tf):
    """
    Performs quintic spline motion on 
    1 DOF robot. Takes in initial values of
    the motion

    Args:
        theta_0 : initial angular position (radians)
        w_0 : initial angular velocity
        a_0 : initial angular acceleration
        t0 : inital time t
        tf : final time t
    """
    T = tf - t0
    A = np.array([[1,0,0,0,0,0],
                  [0,1,0,0,0,0],
                  [0,0,2,0,0,0],
                  [1, T, T**2, T**3, T**4, T**5],
                  [0, 1, 2*T, 3*(T**2), 4*(T**3), 5*(T**4)],
                  [0, 0, 2, 6*T, 12*(T**2), 20*(T**3)] ])

    if (np.linalg.det(A) == 0):
        print("Matrix is not invertible")
        return -1
    values = np.array([[theta_0], [w_0], [a_0], [theta_f], [w_f], [a_f]])
    consts = np.matmul(np.linalg.inv(A), values)
    print(consts)
    a,b,c = consts[0,0],consts[1,0],consts[2,0]
    d,e,f = consts[3,0],consts[4,0],consts[5,0]

    N = 1000 #number of points to plot
    thetas = []
    omegas = []
    accels = []
    times = []
    for i in range(N+1):
        t = t0 + (T/N) * i
        theta = a + b*t + c*(t**2) + d*(t**3) + e*(t**4) + f*(t**5)
        omega = b + 2*c*t + 3*d*(t**2) + 4*e*(t**3) + 5*f*(t**4)
        accel = 2*c + 6*d*t + 12*e*(t**2) + 20*f*(t**3)

        times.append(t)
        thetas.append(theta)
        omegas.append(omega)
        accels.append(accel)

    print("Max position reached = {} radians".format(max(thetas)))
    plt.plot(times, thetas)
    plt.title("Quintic Spline Motion")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (radians)")
    plt.show()

    plt.plot(times, omegas)
    plt.title("Quintic Spline Motion")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rads/s)")
    plt.show()

def problem_2():
    x_pos = [0]
    y_pos = [0]

    theta1 = 45 * np.pi/180
    d2 = 3 * np.sqrt(2)
    theta3 = 45 * np.pi/180
    d4 = 2
    theta5 = 90 * np.pi/180
    d6 = 1

    x1 = d2 * np.cos(theta1)
    y1 = d2 * np.sin(theta1)
    x_pos.append(x1)
    y_pos.append(y1)

    x2 = x1 + d4 * np.cos(theta1 + theta3)
    y2 = y1 + d4 * np.sin(theta1 + theta3)
    x_pos.append(x2)
    y_pos.append(y2)

    x3 = x2 + d6 * np.cos(theta1 + theta3 + theta5)
    y3 = y2 + d6 * np.sin(theta1 + theta3 + theta5)
    x_pos.append(x3)
    y_pos.append(y3)

    label = 'O ({},{}) '.format(round(0,3),round(0,3))
    plt.text(0, 0, label, horizontalalignment='right')
    for i in range(1, len(x_pos)):
        x0,y0 = x_pos[i-1], y_pos[i-1]
        xf,yf = x_pos[i], y_pos[i]
        plt.scatter(x0, y0, marker = 'o')

        plt.scatter(xf, yf, marker='o')
        if i != len(x_pos) - 1:
            if i == 1:
                plt.text(xf, yf, '  A ({},{})'.format(round(xf,3),round(yf,3)))
            if i == 2:
                plt.text(xf, yf, '  B ({},{})'.format(round(xf,3),round(yf,3)))

        plt.plot([x0,xf], [y0,yf])

    label = 'Tip ({},{}) '.format(round(x_pos[-1],3),round(y_pos[-1],3))
    plt.text(x_pos[-1], y_pos[-1], label , horizontalalignment='right')
    plt.xlabel("x (meters)")
    plt.ylabel('y (meters)')
    plt.xlim(-1, 4)
    plt.ylim(-1, 6)
    plt.title("Robot at given config")
    plt.grid()
    plt.show()

def cross_products():
    e_i = np.array([0,0,1])
    p0 = np.array([0,0,0])
    pA = np.array([3,3,0])
    pB = np.array([3,5,0])
    pTip = np.array([2,5,0])

    cross1 = np.cross(e_i, pTip - p0) # For J1
    cross3 = np.cross(e_i, pTip - pA) # For J3
    cross5 = np.cross(e_i, pTip - pB)# For J5


if __name__ == "__main__":
    #problem_1a(0, 1, 0, 0, 0, 1)
    #problem_1b(0,1,0, 0,0,0, 0,1)
    problem_2()
    #cross_products()

    #problem 3
    #R = Rotation.from_quat([0.341363, -0.117017, 0.0915447, 0.928115])
    #print(R.as_matrix())    