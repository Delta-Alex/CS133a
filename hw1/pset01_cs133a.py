import numpy as np
import matplotlib.pyplot as plt

b = 6
c = 5
d = 7
a_vals = [2, 5, 6]
def get_theta(a, phi):
    # get the location of B
    B_x = d + c * np.cos(phi)
    B_y = c * np.sin(phi)

    gamma = np.arctan2(B_y, B_x)
    r = np.sqrt(B_y**2 + B_x**2)
    beta = np.arccos((a**2 + r**2 - b**2) / (2*a*r))

    return [gamma + beta, gamma - beta]

N = 1000

for a in a_vals:
    print("a = {} m.".format(a))
    phi_lst = []
    theta_lst = []
    for i in range(N+1):
        try:
            phi = i * 2 *np.pi/N
            theta1, theta2 = get_theta(a, phi)
            if not np.isnan(theta1):
                #print("(In degrees) phi = {} and theta = {}".format(phi * 180/np.pi, theta1 * 180/np.pi))
                phi_lst.append(phi)
                theta_lst.append(theta1)

                phi_lst.append(phi)
                theta_lst.append(theta1 + 2 *np.pi)

                phi_lst.append(phi)
                theta_lst.append(theta1 - 2 *np.pi)


                phi_lst.append(phi)
                theta_lst.append(theta2)

                phi_lst.append(phi)
                theta_lst.append(theta2 + 2 *np.pi)

                phi_lst.append(phi)
                theta_lst.append(theta2 - 2 *np.pi)
        except:
            pass

    for i in range(N+1):
        try:
            phi = -i * 2 *np.pi/N
            theta1, theta2 = get_theta(a, phi)
            if not np.isnan(theta1):
                phi_lst.append(phi)
                theta_lst.append(theta1)

                phi_lst.append(phi)
                theta_lst.append(theta1 + 2 *np.pi)

                phi_lst.append(phi)
                theta_lst.append(theta1 - 2 *np.pi)


                phi_lst.append(phi)
                theta_lst.append(theta2)

                phi_lst.append(phi)
                theta_lst.append(theta2 + 2 *np.pi)

                phi_lst.append(phi)
                theta_lst.append(theta2 - 2 *np.pi)
        except:
            pass

    plt.scatter(phi_lst, theta_lst)
    plt.xlim((-2 * np.pi, 2*np.pi))
    plt.ylim((-2*np.pi, 2*np.pi))
    plt.title('a = {} m.'.format(a))
    plt.xlabel("Φ (in radians)")
    plt.ylabel("θ (in radians)")
    plt.show()