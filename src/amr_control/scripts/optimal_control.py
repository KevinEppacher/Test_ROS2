#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# Casadi imports
from casadi import *
from casadi.tools import *
# Other imports
import numpy as np


class nMPC(Node):

    def __init__(self, N, dt, xRef, uRef, x0, S, Q, R, uMax, uMin):
        super().__init__('nmpc_node')  # Initialize the Node with a name
        self.N = N
        self.dt = dt
        self.xRef = xRef
        self.uRef = uRef
        self.x0 = x0
        self.Q = Q
        self.R = R
        self.S = S
        self.uMax = uMax
        self.uMin = uMin
        print("nMPC object created")

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')

    def model(self, x, u):
        return None
    
    def cost(self, x, u):
        return None
    
    def solve(self, x):
        return None
    
    def terminalCost(self, x):
        return None
    
    def solve(self, x0):
        return None
    


def main(args=None):
    rclpy.init(args=args)
    controller = nMPC(10, 0.01, None, None, None, None, None, None, None, None)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
