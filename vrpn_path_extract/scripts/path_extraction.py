#!/usr/bin/env python3

print('file init')

import os
import time
import sys
import rclpy
import ros2pkg

from math import sqrt, pow
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 
from rclpy.qos import QoSProfile

class PathExtraction(Node):

    def __init__(self):
        super().__init__('path_extraction')
        
        print('ros init')

        qos_profile = QoSProfile(depth=10)

        # ROS Publisher for DeepRacer
        self.path_rviz_pub = self.create_publisher(Path, '/path_rviz', qos_profile)
        self.path_msg = Path()

        # ROS Subscriber for Optitrack
        self.create_subscription(PoseStamped, "/RigidBody/pose", self.poseCallback, qos_profile)
        
        print('configure start')
        self.configure()
        print('configure end')
        self.is_status = False
        self.prev_x = 0
        self.prev_y = 0
        
        # find directory to save path file
        self.touchPathFile()

        #print('timer start')
        #timer_period = 0.03
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
    #     print('loop start: timer_callback')
    #     try:
    #         #while rclpy.ok():
    #             if self.is_status == True :
    #                 print('saving path')
    #                 self.savePath()
    #     except KeyboardInterrupt:
    #         self.f.close()
                    
    def configure(self):
        self.path_directory_name = self.declare_parameter("path_ext")
        self.path_file_name = self.declare_parameter("extractedPath.csv")
        #self.min_sample_distance = self.declare_parameter("0.15")
        self.min_sample_distance = 0.1

    def touchPathFile(self):
        pkg_path = get_package_share_directory('vrpn_path_extract')
        #full_path = pkg_path + '/' + self.path_directory_name + '/' + self.path_file_name
        abcde = time.time()
        full_path = pkg_path + '/' + "path_ext" + '/' + "pathExtracted_{}.csv".format(abcde)
        
        self.f = open(full_path, 'a')
        
    def poseCallback(self, msg):
        self.is_status = True

        self.path_frame_id = msg.header.frame_id
        self.status_msg = msg.pose.position
        print("done poseCallback")
        if self.is_status == True:
            self.savePath()
        
    def savePath(self):
        x = self.status_msg.x
        y = self.status_msg.y
        z = 0

        # distance between 2 path points
        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

        if distance > self.min_sample_distance:
            data = '{0},{1},{2}\n'.format(x, y, z)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y

            # debug
            print(x, y)

            last_point = PoseStamped()
            last_point.pose.position.x = x
            last_point.pose.position.y = y
            last_point.pose.position.z = 0.0
            last_point.pose.orientation.x = 0.0
            last_point.pose.orientation.y = 0.0
            last_point.pose.orientation.z = 0.0
            last_point.pose.orientation.w = 1.0

            self.path_msg.header.frame_id = self.path_frame_id
            # self.path_msg.header.frame_id = 'map'
            self.path_msg.poses.append(last_point)
            self.path_rviz_pub.publish(self.path_msg)

if __name__ == '__main__':
    rclpy.init()
    try:
        path_extractor = PathExtraction()
        while rclpy.ok():
            print("let's spin")
            rclpy.spin_once(path_extractor, timeout_sec=0.1)

    except KeyboardInterrupt: 
        path_extractor.f.close()
        rclpy.shutdown()
        pass