#!/usr/bin/env python3

print('file init')

import os
import time
import sys
import rclpy
import ros2pkg

from rclpy.clock import Clock
from rclpy.time import Time

from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

class DeepStatus(Node):

    def __init__(self):
        super().__init__('deepracer_pub')
        
        print('ros init')

        qos_profile = QoSProfile(depth=10)

        # ROS Publisher for DeepRacer
        self.status_pub = self.create_publisher(Odometry, '/deep_status', qos_profile)
        self.pose_vel_msg = Odometry()

        # ROS Subscriber for Optitrack
        self.create_subscription(PoseStamped, "/RigidBody/pose", self.poseCallback, qos_profile)
        
        self.is_status = False #RigidBody/pose 정보가 들어오면 True로 아니면 False로
        self.prev_x = 0
        self.prev_y = 0
        self.previous_time = clock.now().nanoseconds
        
    def poseCallback(self, msg):
        self.is_status = True

        self.pose_frame_id = msg.header.frame_id
        self.pose_msg = msg.pose
        print("done poseCallback")
        if self.is_status == True:
            self.distCalc()
        
    def distCalc(self):
        self.current_time = clock.now().nanoseconds
        x = self.pose_msg.position.x
        y = self.pose_msg.position.y

        # distance between 2 path points & velocity calculation
        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
        #print("distance = ", distance)
        time_diff = float(self.current_time - self.previous_time)/float(1000000000.0)
        #print("time_diff = ", time_diff)
        velocity = float(distance/time_diff)
        self.prev_x = x
        self.prev_y = y

        # deepracer status
        self.pose_vel_msg.header.frame_id = self.pose_frame_id
        self.pose_vel_msg.child_frame_id = self.pose_frame_id
        self.pose_vel_msg.pose.pose = self.pose_msg
        self.pose_vel_msg.twist.twist.linear.x = velocity
        print("vel = ", velocity)
        self.status_pub.publish(self.pose_vel_msg)
        self.previous_time = self.current_time

if __name__ == '__main__':
    rclpy.init()
    try:
        clock = Clock()
        status_publisher = DeepStatus()
        while rclpy.ok():
            print("let's spin")
            rclpy.spin_once(status_publisher, timeout_sec=0.1)

    except KeyboardInterrupt: 
        rclpy.shutdown()
        pass
