'''
hw7p3c.py
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
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.pleft  = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        self.phigh = np.array([0.0, 0.5, 0.9]).reshape((-1,1))
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))

        # Initialize the current/starting joint position.
        self.qlast  = self.q0
        self.xd_last = self.p0
        self.Rd_last = self.R0
        self.lam = 20

        self.pub = node.create_publisher(Float64, '/condition', 10)

        # gain for secondary task
        self.lam_s = 10

        # secondary joint goal 
        self.q_goal_s = np.array([-pi/4, -pi/4, pi/2, -pi/2, 0, 0, 0]).reshape((-1,1))


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t >= 8.0:
            return None

        if t < 3.0:
            # Goes to from p0 to pright:
            (s0, s0dot) = goto5(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.array([[0],[0],[0]])

        else:
            t1 = (t-3) % 5.0
            if t1 < 1.25:
                # from pright to phigh
                (sp, spdot) = goto5(t1, 1.25, -1.0,  1.0)
                (sR, sRdot) = goto5(t1, 1.25, 0,  1.0)

                # Use the path variables to compute the trajectory.
                pd = 0.5*(self.phigh+self.pright) + 0.5*(self.phigh-self.pright) * sp
                vd =                            + 0.5*(self.phigh-self.pright) * spdot

                Rd = Roty(-pi/2 * sR)
                wd = ey() * (-pi/2 * sRdot)
            elif t1 < 2.50:
                # from phigh to pleft
                (sp, spdot) = goto5(t1-1.25, 1.25, -1.0,  1.0)
                (sR, sRdot) = goto5(t1-1.25, 1.25, 0,  1.0)

                # Use the path variables to compute the trajectory.
                pd = 0.5*(self.pleft+self.phigh) + 0.5*(self.pleft-self.phigh) * sp
                vd =                            + 0.5*(self.pleft-self.phigh) * spdot

                Rd = Roty(-pi/2) @ Rotz(pi/2 * sR)
                wd = (Roty(-pi/2) @ ez()) * (pi/2 * sRdot)

            elif t1 < 3.75:
                # from pleft to phigh
                (sp, spdot) = goto5(t1-2.50, 1.25, -1.0,  1.0)
                (sR, sRdot) = goto5(t1-2.50, 1.25, 1.0,  0)

                # Use the path variables to compute the trajectory.
                pd = 0.5*(self.phigh+self.pleft) + 0.5*(self.phigh-self.pleft) * sp
                vd =                            + 0.5*(self.phigh-self.pleft) * spdot

                Rd = Roty(-pi/2) @ Rotz(pi/2 * sR)
                wd = (Roty(-pi/2) @ ez()) * (pi/2 * sRdot)
            else:
                # from phigh to pright
                (sp, spdot) = goto5(t1-3.75, 1.25, -1.0,  1.0)
                (sR, sRdot) = goto5(t1-3.75, 1.25, 1.0,  0)

                # Use the path variables to compute the trajectory.
                pd = 0.5*(self.pright+self.phigh) + 0.5*(self.pright-self.phigh) * sp
                vd =                            + 0.5*(self.pright-self.phigh) * spdot

                Rd = Roty(-pi/2 * sR)
                wd = ey() * (-pi/2 * sRdot)


        # Compute the old forward kinematics.
        (ptip, R, Jv, Jw) = self.chain.fkin(self.qlast)

        # Compute the errors
        error_pos = ep(self.xd_last, ptip)
        error_rot = eR(self.Rd_last, R)
        error = np.vstack((error_pos, error_rot))

        # compute qdot
        v = np.vstack((vd,wd))
        A = v + self.lam * error
        J = np.vstack((Jv, Jw))

        # secondary qdot
        qdot_s = self.lam_s * (self.q_goal_s - self.qlast)

        (M, N) = J.shape
        qdot = (np.linalg.pinv(J) @ A) + ( (np.eye(N) - np.linalg.pinv(J) @ J) @ qdot_s)

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
