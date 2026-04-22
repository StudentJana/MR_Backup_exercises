

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MRMove(Node):

    def __init__(self):
        super().__init__('move')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        timer_period_name = 10.0 
        self.declare_parameter('mode', 'demo')
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_name = self.create_timer(timer_period_name, self.timer_callback_name)
        self.timer_callback()

        #Add five parameters for real time changes
        self.declare_parameter('safety_bubble', 0.5)
        self.declare_parameter('safety_distance', 1.5)
        self.declare_parameter('threshold', 2.5)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('turn_speed', 0.3)
        self.declare_parameter('robot_name', 'Indiana Jana')

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_laser,
            10)
        
        self.publisher_name = self.create_publisher(String, 'name', 10)


    def timer_callback(self):
        self.param_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Publishing: "{0}, {1}"'.format(self.cmd.linear.x, self.cmd.angular.z))

    def timer_callback_name(self):
        msg = String()
        msg.data = self.get_parameter('robot_name').get_parameter_value().string_value
        self.publisher_name.publish(msg)
        self.get_logger().info('Publishing: "{0}"'.format(self.get_parameter('robot_name').get_parameter_value().string_value))


    def callback_laser(self, msg: LaserScan):
        self.get_logger().info(self.param_mode)
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
        self.get_logger().info('Wanderer')

        scan = msg #backup data

        #parse arguments
        safety_bubble = self.get_parameter('safety_bubble').get_parameter_value().double_value
        safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        threshold = self.get_parameter('threshold').get_parameter_value().double_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        #using a follow the gap algorithm 

        #clean data
        ranges = np.array(scan.ranges)
        ranges = np.nan_to_num(ranges, posinf=20.0, neginf=0.0)

        bubble_size = int(safety_bubble / scan.angle_increment) 
        
        
        close_indices = np.where(ranges < safety_distance)[0]

        #add safety bubble around all objects that are closer than safety distance
        for idx in close_indices:
            bubble_start = max(0, idx - bubble_size)
            bubble_end = min(len(ranges), idx + bubble_size)
            ranges[bubble_start : bubble_end] = 0.0

        valid_indices = np.where(ranges > threshold)[0]
        
        #no valid inidices turn on spot 
        if len(valid_indices) == 0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = turn_speed
            return

        gaps = []
        current_gap = []

        for i in range(len(valid_indices)):
            if i == 0:
                current_gap.append(valid_indices[i])
            else:
                if valid_indices[i] == valid_indices[i-1] + 1:
                    current_gap.append(valid_indices[i])
                else:
                    gaps.append(current_gap)
                    current_gap = [valid_indices[i]]
        
        if current_gap:
            gaps.append(current_gap)        

        largest_gap = max(gaps, key=len)

        target_idx = int(np.mean(largest_gap))
        center_idx = len(ranges) // 2 
        
        target_angle = (target_idx - center_idx) * scan.angle_increment

        #steering_velocity = np.clip(target_angle, -turn_speed, turn_speed)
        if abs(target_angle)>turn_speed: 
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = turn_speed
            return
        
        self.cmd.angular.z = float(target_angle)

        # Slow down for sharp turns
        if abs(target_angle) > 0.2:
            self.cmd.linear.x = max_speed/3
        else: 
            self.cmd.linear.x = max_speed
