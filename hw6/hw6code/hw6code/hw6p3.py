'''hw6p3.py

   This is the skeleton code for HW6 Problem 3.  Please EDIT.

   This creates a purely rotational movement.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp
import math

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.qlast = np.zeros((3,1))
        (_, self.Rd_last, _, _) = self.chain.fkin(self.qlast)

        # Pick the convergence bandwidth.
        self.lam = 20

        # rotation axis for t>2
        # relative to tip frame, and world frame
        self.e_tip = exyz(0, 0.05, -0.05)
        self.e_w =  exyz(0.05, 0.05, 0)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['pan', 'tilt', 'roll']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t >= 15:
            return None

        # Choose the alpha/beta angles based on the phase.
        if t <= 2.0:
            # Part A (t<=2):
            (alpha, alphadot) = goto(t, 2, 0, -np.pi/2)
            (beta,  betadot)  = (0.0, 0.0)

            Rd = Roty(alpha)
            wd = ey() * alphadot
        else:
            # Part B (t>2):
            (alpha, alphadot) = (-np.pi/2, 0)
            beta = t - 3 + math.e**(2-t)
            betadot = 1 - math.e**(2-t)

            # Compute the desired rotation and angular velocity.
            Rd = Roty(alpha) @ Rote(self.e_tip, beta)
            wd = self.e_w * betadot

        # Compute the old forward kinematics.
        (_, R, _, Jw) = self.chain.fkin(self.qlast)

        # Compute the inverse kinematics
        error = eR(self.Rd_last, R)
        A = wd + self.lam * error
        qdot = np.linalg.pinv(Jw) @ A

        # Integrate the joint position.
        q = self.qlast + dt * qdot

        # Save the data needed next cycle.
        self.qlast = q
        self.Rd_last = Rd

        # Return the position and velocity as python lists.
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
