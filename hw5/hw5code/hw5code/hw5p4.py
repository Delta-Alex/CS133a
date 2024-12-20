'''hw5p4.py

   This is skeleton code for HW5 Problem 4.  Please EDIT.

   This moves the tip in a straight line (tip spline), then returns in
   a joint spline.

'''

import rclpy
import numpy as np

from math                       import pi, sin, cos, acos, atan2, sqrt, fmod

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TrajectoryUtils    import goto, spline, goto5, spline5

# Grab the fkin and Jac from P1.
from hw5code.hw5p1 import fkin, Jac


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Define the known tip/joint positions.
        self.qA = np.radians(np.array([ 0, 60, -120])).reshape(3,1)
        self.xA = fkin(self.qA)

        self.qD = None
        self.xD = np.array([0.5, -0.5, 1.0]).reshape(3,1)

        # Select the leg duration.
        self.T = 3.0

        # Initialize the parameters and anything stored between cycles!
        self.l = 20 #lambda
        self.q_prev = self.qA
        self.zero_vec = np.array([0, 0, 0]).reshape(3,1)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        # End after one cycle.
        # uncomment to keep going indefinitely
        if (t > 2*self.T):
           return None
        
        # First modulo the time by 2 legs.
        t = fmod(t, 2*self.T)

        # reset after every cycle
        if (t < 0.01):
            print('reset')
            self.q_prev = self.qA
            q = self.q_prev
            qdot = self.zero_vec
            return (q.flatten().tolist(), qdot.flatten().tolist())

        # COMPUTE THE MOTION.
        # from A to D
        if (t < 3.0):
            # desired position
            xd_prev, vd_prev = spline(t-dt, self.T, self.xA, self.xD, self.zero_vec, self.zero_vec)
            xd, vd = spline(t, self.T, self.xA, self.xD, self.zero_vec, self.zero_vec)
            error = xd_prev - fkin(self.q_prev)
            xr_dot = vd + self.l * error
            qdot = np.matmul(np.linalg.inv(Jac(self.q_prev)), xr_dot)
            q = self.q_prev + dt * qdot
            self.q_prev = q

        # from D to A
        else:
            # self.q_prev is the joint config for point D
            (q, qdot) = spline(t-3.0, self.T, self.q_prev, self.qA, self.zero_vec, self.zero_vec)

        # Return the position and velocity as python lists!
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
