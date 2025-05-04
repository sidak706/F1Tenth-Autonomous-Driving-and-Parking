#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import numpy as np
import math
import time

from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge # ROS bridge for opencv library to handle images
from sensor_msgs.msg import Image 


import time


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        
        # Publish drive commands
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Subscribe to LIDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Subscribe to safety flag from safety node
        self.safety_subscription = self.create_subscription(
            Bool,
            'safety_flag',
            self.safety_callback,
            10)

        self.scan_subscription
        self.safety_subscription

        # Init values of the car
        self.speed = 0.0
        self.is_safe = True
        self.found_obstacle = False
        self.emergency_stop = False
        self.lidar_running = False

        # 0 --> Getting to the spot
        # 1 --> Backing
        # 2 --> Wait state
        # 3 --> Turning in
        # 4 --> Straight and stop
        self.state = 0


    # Remove noise
    def process_lidar(self, ranges, range_max):
        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                ranges[i] = 0
            if math.isinf(ranges[i]):
                ranges[i] = range_max

        return ranges

    
    def safety_callback(self, msg):
        self.is_safe = msg.data
        print(f"MSG DATA: {msg.data}")
        if not self.is_safe:
            self.get_logger().warn("Safety flag received: UNSAFE")
            self.emergency_stop = True
            

    def scan_callback(self, scan_msg):
        """LIDAR callback function"""
        # If scan_callback is called, lidar is attached, so raise 
        # the lidar_running flag
        self.lidar_running = True

        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        process_ranges = self.process_lidar(ranges, range_max)
        radius = 120
        length = len(process_ranges)
        closest = 9999
        index = None

        if self.state == 0:
            self.get_to_parking_lot(scan_msg, ranges)
        if self.state == 1: 
            self.align_to_car(scan_msg, process_ranges)

        elif self.state == 2: 
            time.sleep(1.5)
            self.state = 3

        elif self.state == 3: 
            self.turn_in(scan_msg, process_ranges)

        elif self.state == 4:
           self.straight_and_stop()
    
    def get_to_parking_lot(self, scan_msg, ranges):
        """Drive until parking spot is detected"""
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        v = 0.7

        start_angle = math.radians(100)
        end_angle = math.radians(130)

        start_index = (int) ((start_angle - angle_min) / (angle_increment))
        end_index = (int) ((end_angle - angle_min) / (angle_increment))

        total = (float) (end_index - start_index)

        count = 0

        for i in range(start_index, end_index):
            if ranges[i] < 0.7:
               count += 1

        if count/total >= 0.4: # checking if enough points are "close" enough
            self.found_obstacle = True
        elif self.found_obstacle and not count/total >= 0.4:
            self.state = 1
            self.found_obstacle = False
            v = 0.0

        # Publish message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.05
        msg.drive.speed = v

        self.publisher_.publish(msg)


    def get_min_distance(self, start_i, end_i, process_ranges):
        """Helper to get minimum value in certain range of values"""
        min_dist = 9999
        for i in range(start_i, end_i):
            min_dist = min(min_dist, process_ranges[i])

        return min_dist


    def align_to_car(self, scan_msg, ranges):
        """Keep backing up until the car is in the middle of the spot next
           to the parking spot
        """
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment

        start_angle = math.radians(50)
        end_angle = math.radians(105)
        
        start_index = (int) ((start_angle - angle_min) / (angle_increment))
        end_index = (int) ((end_angle - angle_min) / (angle_increment))

        min_dist = 9999
        for i in range(start_index, end_index, 1):
            if(ranges[i] < min_dist):
                min_dist = ranges[i]

        # min_dist = self.get_min_distance(start_index, end_index, ranges)
        self.get_logger().info(f" min distance: {min_dist}")

        # keep reversing
        if min_dist > 0.55:
            v = -0.7
            msg = AckermannDriveStamped()
            msg.drive.speed = v
            msg.drive.steering_angle = -0.25
            self.publisher_.publish(msg)
        else:
            v = 0.0
            msg = AckermannDriveStamped()
            msg.drive.speed = v
            msg.drive.steering_angle = 0.0
            self.publisher_.publish(msg)
            self.state = 2


    def turn_in(self, scan_msg, ranges):
        """Turn the car into the parking spot"""
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        v = 0.5

        start_angle = math.radians(-30)
        end_angle = math.radians(30)

        start_index = (int) ((start_angle - angle_min) / (angle_increment))
        end_index = (int) ((end_angle - angle_min) / (angle_increment))

        total = (float) (end_index - start_index)

        count = 0

        for i in range(start_index, end_index):
            if ranges[i] < 0.40:
               count += 1

        # Need 40% of points to be blocked to be considered "in" the spot
        if count/total >= 0.4:
            v = 0.0
            self.state = 4

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.5
        msg.drive.speed = v

        self.publisher_.publish(msg)

    def straight_and_stop(self):
        """Straighten the car and stop"""
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    follower = FollowerNode()
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
