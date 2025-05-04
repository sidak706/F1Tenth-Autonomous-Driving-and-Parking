#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import numpy as np
import math
import time

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
       
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
       
        self.scan_subscription
        self.odom_subscription
        self.speed = 0.0

    # Remove noise
    def process_lidar(self, ranges, range_max):

        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                ranges[i] = 0
            if math.isinf(ranges[i]):
                ranges[i] = range_max

        return ranges

    # Find max length of sequence of consectuive non-zeros among free space points
    def find_max_gap(self, free_space_ranges):
        curr_max_start_index = 0
        curr_max_length = 0
        curr_max_end = 0
        curr_start_index = 0
        curr_length = 0

        for i in range(len(free_space_ranges)):
            if(free_space_ranges[i] > 3.2):
                curr_length += 1
                if(curr_length > curr_max_length):
                    curr_max_length = curr_length
                    curr_max_start_index = curr_start_index
                    curr_max_end = i
            else:
                curr_length = 0
                curr_start_index = i + 1
               
        return curr_max_start_index, curr_max_end

    # Choose furthest point in free space
    def find_best_point(self, start, end, free_space_ranges):
        # dist = free_space_ranges[start]
        # point = start

        # for i in range(start + 1, end + 1):
        #     if free_space_ranges[i] > dist:
        #         dist = free_space_ranges[i]
        #         point = i

        # return point

        return (start + end) // 2


    def odom_callback(self, odom_msg):
        odom_msg
        
        # twist = odom_msg.twist.twist # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        # v = twist.linear.x # Linear velocity, forward direction
         
        # self.speed = v

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        process_ranges = self.process_lidar(ranges, range_max)
        radius = 40
        length = len(process_ranges)
        closest = 9999
        index = None

        # Finding the closest point
        for i in range(length):
            if process_ranges[i] < closest:
                closest = process_ranges[i]
                index = i

        # If no valid closest point is found, stop the robot
        if index is None:
            self.get_logger().warn("No valid closest point found. Stopping the robot.")
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.publisher_.publish(msg)
            return

        # Creating a bubble around the closest point
        for j in range(index - radius, index + radius + 1):
            if j >= 0 and j < length:
                process_ranges[j] = 0

        start_i, end_i = self.find_max_gap(process_ranges)

        best_point = self.find_best_point(start_i, end_i, process_ranges)

        # Calculating the angle
        angle = scan_msg.angle_min + (best_point * scan_msg.angle_increment)

        v = 3.0

        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = angle

        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    follower = FollowerNode()
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()