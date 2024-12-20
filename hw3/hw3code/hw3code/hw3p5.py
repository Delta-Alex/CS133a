'''
hw3p5.py

   This is a skeleton for HW3 Problem 5.  Please EDIT.

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np

from math               import pi, sin, cos, acos, atan2, sqrt, fmod

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self):
        #### PRECOMPUTE ANY DATA YOU MIGHT NEED.
        self.init_pos = True
        self.timer = 0
        self.flag = 0
        self.wait = False

        # The four solutions
        self.A = [0.6747409422235527, 0.5081507901910796, 0.6435011087932847]
        self.B = [0.6747409422235527, 1.1516518989843643, -0.6435011087932847]
        self.C = [-2.4668517113662403, 2.633441863398714, -0.6435011087932847]
        self.D = [-2.4668517113662403, 1.9899407546054289, 0.6435011087932847]

        self.q_lst = [self.A, self.B, self.C, self.D]

        self.qdot_AB = [self.B[0] - self.A[0], self.B[1] - self.A[1], self.B[2] - self.A[2]]
        self.qdot_BC = [self.C[0] - self.B[0], self.C[1] - self.B[1], self.C[2] - self.B[2]]
        self.qdot_CD = [self.D[0] - self.C[0], self.D[1] - self.C[1], self.D[2] - self.C[2]]
        self.qdot_DA = [self.A[0] - self.D[0], self.A[1] - self.D[1], self.A[2] - self.D[2]]

        self.qdots = [self.qdot_AB, self.qdot_BC, self.qdot_CD, self.qdot_DA]

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        #### COMPUTE THE POSITION AND VELOCITY VECTORS AS A FUNCTION OF TIME.        
        #initialize robot to configuration A
        if self.init_pos:
            #set the position to A
            q = self.A
            qdot = [0.0001,0.0001,0.0001]
            self.init_pos = False
            return (q,qdot)
        
        if self.wait:
            qdot = [0.0001,0.0001,0.0001]
            q = self.q_lst[(self.flag + 1)%4]
            self.timer += dt
            if self.timer >= 0.50:
                self.wait = False
                self.flag = (self.flag + 1) % 4
                self.timer = 0

            return (q,qdot)

        else:
            self.timer += dt
            qdot = self.qdots[self.flag]
            q = []
            q.append(self.q_lst[self.flag][0] + qdot[0] * self.timer)
            q.append(self.q_lst[self.flag][1] + qdot[1] * self.timer)
            q.append(self.q_lst[self.flag][2] + qdot[2] * self.timer)

            if self.timer >= 1:
                q = self.q_lst[(self.flag + 1)%4]
                qdot = [0.0001,0.0001,0.0001]
                self.wait = True
                self.timer = 0
                return (q,qdot)

            return (q,qdot)


#
#   Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Take the node name and the update frequency as arguments.
#
class Generator(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it 'generator'
        super().__init__(name)

        # Set up the trajectory.
        self.trajectory = Trajectory()
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Create a timer to trigger calculating/sending commands.
        self.timer     = self.create_timer(1/float(rate), self.update)
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.t         = - self.dt
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Update - send a new joint command every time step.
    def update(self):
        # Grab the current time (from the ROS clock, since 1970).
        now = self.get_clock().now()

        # To avoid any time jitter enforce a constant time step in
        # integrate to get the current time.
        self.t += self.dt

        # Compute the desired joint positions and velocities for this time.
        (q, qdot) = self.trajectory.evaluate(self.t, self.dt)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time for ROS
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates.
    generator = Generator('generator', 100)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
