'''hw5p2.py

   This is skeleton code for HW5 Problem 2.  Please EDIT.

   Implement the Newton-Raphson for seven target points.

'''

from turtle import distance
import numpy as np

# Grab the fkin and Jac from P1.
from hw5code.hw5p1 import fkin, Jac
from matplotlib import pyplot as plt


#
#  Utilities
#
# 360 deg wrapping:
def wraps(q):
    return np.round(q / (2*np.pi))

def unwrapped(q):
    return q - np.round(q / (2*np.pi)) * (2*np.pi)

# 3 DOF Multiplicities - return True of False!
def elbow_up(q):
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    return np.sin(q[2,0]) < 0.0

def front_side(q):
    l1, l2  = 1, 1
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    return l1 * np.cos(q[1,0]) + l2 * np.cos(q[1,0] + q[2,0]) > 0.0



#
#  Newton Raphson
#
def newton_raphson(xgoal):
    # Collect the distance to goal and change in q every step!
    xdistance = []
    qstepsize = []

    # Set the initial joint value guess.
    q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)
    min_error = 10e-12
    steps = 0
    converged = False

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!
    for i in range(0, 20):
        x_diff = xgoal - fkin(q)
        dist_error = np.linalg.norm(x_diff)
        xdistance.append(dist_error)
        q_next = q + np.matmul(np.linalg.inv(Jac(q)), x_diff)
        qstepsize.append(np.linalg.norm(q_next - q))
        if dist_error < min_error:
            converged = True
            print("Converged for {}".format(xgoal))
            print("Steps required: {}".format(steps))
            print("Final q = {}".format(q))
            elbow = "elbow down"
            side = "back side"
            if elbow_up(q):
                elbow = "elbow up"
            if front_side(q):
                side = "front side"
            print("Corresponds to an {} and {} solution".format(elbow, side))
            print("Wraps by 360 deg: {}".format(wraps(q)))
            print("-"*50)
            break
        q = q_next
        steps += 1

    if not converged:
        print("Did not converge for {}".format(xgoal))
        print("-" * 50)

    # Create a plot of x distances to goal and q step sizes, for N steps.
    N = 20
    xdistance = xdistance[:N+1]
    qstepsize = qstepsize[:N+1]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.plot(range(len(xdistance)), xdistance)
    ax2.plot(range(len(qstepsize)), qstepsize)

    ax1.set_title(f'Convergence Data for {xgoal.T}')
    ax2.set_xlabel('Iteration')

    ax1.set_ylabel('Task Distance to Goal')
    ax1.set_ylim([0, max(xdistance)])
    ax1.set_xlim([0, N])
    ax1.set_xticks(range(N+1))
    ax1.grid()

    ax2.set_ylabel('Joint Step Size')
    ax2.set_ylim([0, max(qstepsize)])
    ax2.set_xlim([0, N])
    ax2.set_xticks(range(N+1))
    ax2.grid()

    plt.show()


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Prcess each target (goal position).
    for xgoal in [np.array([0.5,  1.0, 0.5]).reshape((3,1)), 
                  np.array([1.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([2.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([0.0, -1.0, 0.5]).reshape((3,1)),
                  np.array([0.0, -0.6, 0.5]).reshape((3,1)),
                  np.array([0.5, -1.0, 0.5]).reshape((3,1)),
                  np.array([-1.0, 0.0, 0.5]).reshape((3,1))]:
        newton_raphson(xgoal)

if __name__ == "__main__":
    main()
