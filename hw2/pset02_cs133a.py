import numpy as np
import matplotlib.pyplot as plt


def get_thetas(min, max, dtheta):
    theta_lst = []
    theta = min
    while theta < max:
        theta_lst.append(theta)
        theta += dtheta
        
    if theta_lst[-1] != max:
        theta_lst.append(max)

    return theta_lst


def differentiate(y_lst, x_lst):
    # Numerical differetiation of y with respect to x
    dy_dx = []
    for i in range(1, len(y_lst)):
        dy_dx.append((y_lst[i] - y_lst[i-1])/ (x_lst[i] - x_lst[i-1]))

    return dy_dx


def problem_2(th1_range, th2_range, th3_range):
    l1, l2, l3 = 5, 2, 1

    (th1_min, th1_max) = th1_range
    (th2_min, th2_max) = th2_range
    (th3_min, th3_max) = th3_range

    dtheta = 0.05
    th1_lst = get_thetas(th1_min, th1_max, dtheta)
    th2_lst = get_thetas(th2_min, th2_max, dtheta)
    th3_lst = get_thetas(th3_min, th3_max, dtheta)

    px = []
    py = []

    for theta1 in th1_lst:
        for theta2 in th2_lst:
            for theta3 in th3_lst:
                x1 = l1 * np.cos(theta1)
                y1 = l1 * np.sin(theta1)

                x2 = x1 + l2 * np.cos(theta1 + theta2)
                y2 = y1 + l2 * np.sin(theta1 + theta2)

                x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
                y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)

                px.append(x3)
                py.append(y3)

    plt.scatter(px, py)
    plt.title('(x,y) position of tip point')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid()
    plt.show()

def problem_3():
    l1, l2, l3 = 6, 1, 0.50
    w1, w2, w3 = 1, -4, 12
    duration = 1000 #in seconds
    dt = 0.05
    t = 0 #current time

    px = []
    py = []
    while t < duration:
        theta1 = w1 * t
        theta2 = w2 * t + np.pi
        theta3 = w3 * t + np.pi

        x1 = l1 * np.cos(theta1)
        y1 = l1 * np.sin(theta1)

        x2 = x1 + l2 * np.cos(theta1 + theta2)
        y2 = y1 + l2 * np.sin(theta1 + theta2)

        x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)

        px.append(x3)
        py.append(y3)
        t += dt

    plt.scatter(px, py)
    plt.title('(x,y) position of tip point')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid()
    plt.show()


def problem_4():
    duration = 1000 #in seconds
    dt = 0.05
    t = 0 #current time

    px = []
    py = []

    while t < duration:
        theta_pan = (np.pi/3) * np.sin(2*t)
        theta_tilt = (np.pi/3) * np.sin(t) - (np.pi/9) * np.cos(6*t)
        
        x = -2 * np.tan(theta_pan)
        y = 2 * np.tan(theta_tilt)

        px.append(x)
        py.append(y)
        t += dt

    plt.scatter(px, py)
    plt.title('Screen')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid()
    plt.show()


def problem_5():
    duration = 2 * np.pi #in seconds
    dt = 0.001 #increments
    
    x = 0.05
    l1,l2 = 1,1

    theta1_lst, theta2_lst = [], []
    t_lst = get_thetas(0, duration, dt)
    y_lst = []
    for t in t_lst:
        y = np.cos(t)
        r_sq = x**2 + y**2 #r_sq = r^2
        theta2 = np.arccos((r_sq - l1**2 - l2**2)/(2*l1*l2))
        if theta2 > 0:
            theta2 = -theta2

        if theta2 < 0:
            theta1 = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))
            theta1_lst.append(theta1)
            theta2_lst.append(theta2)
            y_lst.append(y)

    figure, axis = plt.subplots(2,2) 
    figure.tight_layout(pad=2.0)

    axis[0,0].scatter(t_lst,  theta1_lst)
    axis[0,0].set_title('θ_1 vs time')
    axis[0,0].set_xlabel("time (s)")
    axis[0,0].set_ylabel("θ_1 (in radians)")
    axis[0,0].grid()

    axis[0,1].scatter(t_lst[0:-1],  differentiate(theta1_lst, t_lst))
    axis[0,1].set_title('dθ_1/dt vs time')
    axis[0,1].set_xlabel("time (s)")
    axis[0,1].set_ylabel("Derivative of θ_1 wrt time")
    axis[0,1].grid()


    axis[1,0].scatter(t_lst,  theta2_lst)
    axis[1,0].set_title('θ_2 vs time')
    axis[1,0].set_xlabel("time (s)")
    axis[1,0].set_ylabel("θ_2 (in radians)")
    axis[1,0].grid()

    axis[1,1].scatter(t_lst[0:-1],  differentiate(theta2_lst, t_lst))
    axis[1,1].set_title('dθ_2/dt vs time')
    axis[1,1].set_xlabel("time (s)")
    axis[1,1].set_ylabel("Derivative of θ_2 wrt time")
    axis[1,1].grid()

    print("Max value of θ_1 = {}".format(max(theta1_lst)))
    i = theta1_lst.index(max(theta1_lst))
    print("Min value of θ_2 = {}".format(theta2_lst[i]))
    print("Corresponding y = {}".format(y_lst[i]))
    plt.show()

    # plot the robot config for max theta_1
    theta1 = max(theta1_lst)
    theta2 = theta2_lst[i]
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    plt.scatter(x1, y1, marker = 'o')
    plt.scatter(x2, y2, marker='o')
    plt.text(x2, y2, 'P')
    plt.plot([0,x1], [0,y1])
    plt.plot([x1,x2], [y1,y2])
    plt.xlabel("x")
    plt.ylabel('y')
    plt.title("Config for max theta_1")
    plt.grid()
    plt.show()

    # plot the robot config for Y = 0
    x = 0.05
    y = 0
    r_sq = x**2 + y**2 #r_sq = r^2
    theta2 = np.arccos((r_sq - l1**2 - l2**2)/(2*l1*l2))
    if theta2 > 0:
        theta2 = -theta2

    if theta2 < 0:
        theta1 = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))

    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    plt.scatter(x1, y1, marker = 'o')
    plt.scatter(x2, y2, marker='o')
    plt.text(x2, y2, 'P')
    plt.plot([0,x1], [0,y1])
    plt.plot([x1,x2], [y1,y2])
    plt.xlabel("x")
    plt.ylabel('y')
    plt.title("Config for y = 0")
    plt.grid()
    plt.show()
    

def problem_6_fkin(pan, theta1, theta2, l1, l2):
    x = 0
    y = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    z = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    #print("On yz plane P = ({}, {}, {})".format(x,y,z))

    # apply rotation about z
    R_z = np.matrix([[np.cos(pan), -np.sin(pan), 0],
                    [np.sin(pan),  np.cos(pan), 0],
                    [     0,            0,      1]])
    P = np.matmul(R_z, [x,y,z])
    return P

def problem_6_ikin(x,y,z, l1, l2):

    # find pan angle
    pan = np.arctan2(-x, y)
    #print("Calculated Pan = {}".format(pan))

    #rotate P back to yz plane
    R_z = np.matrix([[np.cos(pan), -np.sin(pan), 0],
                    [np.sin(pan),  np.cos(pan), 0],
                    [     0,            0,      1]])

    P = np.matmul(R_z.T, [x,y,z])
    # solve for theta 1 and theta2
    #print("Calculated P on yz plane = ({}, {}, {})".format(P[0,0],P[0,1],P[0,2]))

    # now on yz plane
    y, z = P[0,1], P[0,2]
    r_sq = y**2 + z**2

    #two possible value for theta 2
    theta2 = [np.arccos((r_sq - l1**2 - l2**2)/ (2*l1*l2)), -np.arccos((r_sq - l1**2 - l2**2)/ (2*l1*l2))]
    theta1 = []
    theta1.append( np.arctan2(z, y) - np.arctan2(l2*np.sin(theta2[0]), l1 + l2*np.cos(theta2[0])) )
    theta1.append( np.arctan2(z, y) - np.arctan2(l2*np.sin(theta2[1]), l1 + l2*np.cos(theta2[1])) )

    return pan, theta1, theta2




def problem_6_fkin(pan, theta1, theta2, l1, l2):
    x = 0
    y = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    z = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    #print("On yz plane P = ({}, {}, {})".format(x,y,z))

    # apply rotation about z
    R_z = np.matrix([[np.cos(pan), -np.sin(pan), 0],
                    [np.sin(pan),  np.cos(pan), 0],
                    [     0,            0,      1]])
    P = np.matmul(R_z, [x,y,z])
    return P


def problem_6_ikin_2(x,y,z, l1, l2):

    # for testing purposes (does not work)

    # find pan angle
    pan = np.arctan2(-x, y)
    #print("Calculated Pan = {}".format(pan))

    #rotate P back to yz plane
    #R_z = np.matrix([[np.cos(pan), -np.sin(pan), 0],
    #                [np.sin(pan),  np.cos(pan), 0],
    #                [     0,            0,      1]])

    #P = np.matmul(R_z.T, [x,y,z])
    #y = (-x*np.sin(pan) + y *np.cos(pan))
    y = np.sqrt(x**2 + y**2)

    # now on yz plane
    #y, z = P[0,1], P[0,2]
    r_sq = y**2 + z**2

    #two possible value for theta 2
    theta2 = [np.arccos((r_sq - l1**2 - l2**2)/ (2*l1*l2)), -np.arccos((r_sq - l1**2 - l2**2)/ (2*l1*l2))]
    theta1 = []
    theta1.append( np.arctan2(z, y) - np.arctan2(l2*np.sin(theta2[0]), l1 + l2*np.cos(theta2[0])) )
    theta1.append( np.arctan2(z, y) - np.arctan2(l2*np.sin(theta2[1]), l1 + l2*np.cos(theta2[1])) )

    return pan, theta1, theta2


# case when theta2 == 2pi or 0
def problem_6_special_case(x,y,z, l1, l2):
    pan = np.arctan2(-x, y)
    R_z = np.matrix([[np.cos(pan), -np.sin(pan), 0],
                    [np.sin(pan),  np.cos(pan), 0],
                    [     0,            0,      1]])

    #P = np.matmul(R_z.T, [x,y,z])
    #y, z = P[0,1], P[0,2]
    y = np.sqrt(x**2 + y**2)
    theta1 = [np.arctan2(z, y), np.arctan2(z, y)]
    theta2 = [0, 0]
    return pan, theta1, theta2



if __name__ == "__main__":

    #problem_2((0, np.pi), (0, 2*np.pi), (0, 2*np.pi)) #part a
    #problem_2((0, np.pi), (0, np.pi), (0, np.pi)) #part b
    #problem_3()
    #problem_4()
    #problem_5()



    l1 = 1
    l2 = 1
    p,th1,th2 = problem_6_ikin_2(-0.80, 1.0, 1.4, l1, l2)

    P_calc = problem_6_fkin(p, th1[0], th2[0], l1, l2)
    print("({}, {}, {}) -> {}".format(p, th1[0], th2[0], P_calc))

    P_calc = problem_6_fkin(p, th1[1], th2[1], l1, l2)
    print("({}, {}, {}) -> {}".format(p, th1[1], th2[1], P_calc))

    P_calc = problem_6_fkin(p - np.pi, np.pi - th1[0], -th2[0], l1, l2)
    print("({}, {}, {}) -> {}".format(p - np.pi, np.pi - th1[0], -th2[0], P_calc))

    P_calc = problem_6_fkin(p - np.pi, np.pi - th1[1], -th2[1], l1, l2)
    print("({}, {}, {}) -> {}".format(p - np.pi, np.pi - th1[1], -th2[1], P_calc))

    '''
    # checking 4 multiplicities, with forward kinematics and inverse kinematics
    pans = get_thetas(0, 2 * np.pi, 0.1)
    theta_1_lst = get_thetas(0, 2 * np.pi, 0.1)
    theta_2_lst =  get_thetas(0, 2 * np.pi, 0.1)

    for pan in pans:
        for theta_1 in theta_1_lst:
            for theta_2 in theta_2_lst:
                P = problem_6_fkin(pan, theta_1, theta_2, l1, l2)

                if theta_2 == 0 or theta_2 == 2*np.pi:
                    p,th1,th2 = problem_6_special_case(P[0,0], P[0,1], P[0,2], l1, l2)
                    P_calc = problem_6_fkin(p, th1[0], th2[0], l1, l2)
                    v = np.asarray((P - P_calc))
                    err = np.dot(v[0], v[0])
                    #print("Calculated P = {}".format(P_calc))

                    P_calc = problem_6_fkin(p, th1[1], th2[1], l1, l2)
                    v = np.asarray((P - P_calc))
                    err = np.dot(v[0], v[0])
                    #print("Calculated P = {}".format(P_calc))

                    P_calc = problem_6_fkin(p + np.pi, np.pi - th1[0], -th2[0], l1, l2)
                    v = np.asarray((P - P_calc))
                    err = np.dot(v[0], v[0])
                    #print("Calculated P = {}".format(P_calc))

                    P_calc = problem_6_fkin(p + np.pi, np.pi - th1[1], -th2[1], l1, l2)
                    v = np.asarray((P - P_calc))
                    err = np.dot(v[0], v[0])
                    #print("Calculated P = {}".format(P_calc))

                    print("Case Max err = {}".format(err))
                    if err >= 0.1 or np.isnan(err):
                        print("OH NO")
                        print("{}, {}, {}".format(pan, theta_1, theta_2))
                        break
                    continue
                #print("-"*50)
                #print("Rotated P = ({},{},{})".format(P[0,0], P[0,1], P[0,2]))
                #print(" ")
                p,th1,th2 = problem_6_ikin_2(P[0,0], P[0,1], P[0,2], l1, l2)

                P_calc = problem_6_fkin(p, th1[0], th2[0], l1, l2)
                v = np.asarray((P - P_calc))
                err = np.dot(v[0], v[0])
                #print("Calculated P = {}".format(P_calc))

                P_calc = problem_6_fkin(p, th1[1], th2[1], l1, l2)
                v = np.asarray((P - P_calc))
                err = np.dot(v[0], v[0])
                #print("Calculated P = {}".format(P_calc))

                P_calc = problem_6_fkin(p + np.pi, np.pi - th1[0], -th2[0], l1, l2)
                v = np.asarray((P - P_calc))
                err = np.dot(v[0], v[0])
                #print("Calculated P = {}".format(P_calc))

                P_calc = problem_6_fkin(p + np.pi, np.pi - th1[1], -th2[1], l1, l2)
                v = np.asarray((P - P_calc))
                err = np.dot(v[0], v[0])
                #print("Calculated P = {}".format(P_calc))
                print("Max err = {}".format(err))
                
                if err >= 0.1 or np.isnan(err):
                    print("OH NO")
                    print("{}, {}, {}".format(pan, theta_1, theta_2))
                    break
        '''

        

        
 

