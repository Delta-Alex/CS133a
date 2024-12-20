'''
hw7p4c.py
'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain
from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 46.5675, 0, -93.1349, 0, 0, 46.5675]).reshape((-1,1)))
        self.p0 =  np.array([0, 0.7, 0.6]).reshape((-1,1))
        self.R0 = Reye()

        # Initialize the current/starting joint position.
        self.qlast  = self.q0
        self.xd_last = self.p0
        self.Rd_last = self.R0
        self.lam = 20
        self.lam_s = 10

        self.pub = node.create_publisher(Float64, '/condition', 10)
        self.gamma = 0.1


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t >= 8.0:
            return None

        pd = np.array([0, 0.95-0.25*np.cos(t), 0.60+0.25*np.sin(t)]).reshape((-1,1))
        vd = np.array([0, 0.25*np.sin(t), 0.25*np.cos(t)]).reshape((-1,1))     

        Rd = Reye()
        wd = np.array([0,0,0]).reshape((-1,1))


        # Compute the old forward kinematics.
        (ptip, R, Jv, Jw) = self.chain.fkin(self.qlast)

        # Compute the errors
        error_pos = ep(self.xd_last, ptip)
        error_rot = eR(self.Rd_last, R)
        error = np.vstack((error_pos, error_rot))

        v = np.vstack((vd,wd))
        A = v + self.lam * error
        J = np.vstack((Jv, Jw))

        theta4 = self.qlast[3,0]
        qdot_s = np.array([0,0,0, self.lam_s * (-pi/2 - theta4) ,0,0,0]).reshape((-1,1))

        #get weighted psuedo inverse
        JT = np.transpose(J)
        JW_pinv = JT @ np.linalg.inv(J @ JT + (self.gamma**2) * np.eye(6)) 

        # compute qdot
        qdot = JW_pinv @ A  + ((np.eye(7) - JW_pinv @ J) @ qdot_s)

        # Integrate the joint position.
        q = self.qlast + dt * qdot

        # Save the data needed next cycle.
        self.qlast = q
        self.xd_last = pd
        self.Rd_last = Rd

        L = 0.4
        Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1]) @ J
        condition = np.linalg.cond(Jbar)
        msg = Float64()
        msg.data = condition
        self.pub.publish(msg)

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
