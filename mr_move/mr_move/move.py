

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MRMove(Node):

    def __init__(self):
        super().__init__('move')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.declare_parameter('mode', 'demo')
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_callback()

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_laser,
            10)

    def timer_callback(self):
        self.param_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Publishing: "{0}, {1}"'.format(self.cmd.linear.x, self.cmd.angular.z))


    def callback_laser(self, msg: LaserScan):
        if(self.param_mode == 'demo'):
            self.move_demo(msg)
        elif(self.param_mode == 'wanderer'):
            self.move_wanderer(msg)
        else: 
            self.cmd = Twist()
            self.get_logger().info('Unknown mode value "%s"' % self.param_mode)


    def move_demo(self, msg: LaserScan):
        self.get_logger().info('Demo')
        nr_of_scans = len(msg.ranges)
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0
        if(nr_of_scans > 0):
            if(msg.ranges[round(nr_of_scans/2)] < 1. ):
                self.cmd.linear.x = 0.0
                if(msg.ranges[round(nr_of_scans/4)] < msg.ranges[round(3*nr_of_scans/4)]  ):
                    self.cmd.angular.z = +0.2
                else:
                    self.cmd.angular.z = -0.2

    def move_wanderer(self, msg: LaserScan):
        pass # do nothing
        # self.get_logger().info('Wanderer')
