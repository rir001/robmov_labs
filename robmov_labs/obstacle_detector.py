#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msg.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
from threading import Thread
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__( 'obstacle_detector' )
        self.subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        # self.publisher = self.create_publisher(Vector3, 'occupancy_state', 10)
        self.bridge = CvBridge()

    def depth_cb( self, data ):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2( data )
        
    def mostrar(self):
        while(True):
            cv2.imshow('frame1', self.current_cv_depth_image) 
            
            if cv2.waitKey(1) & 0xFF == 27: 
                break
        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    thread_screen = Thread(target=node.mostrar)
    thread_screen.start()
    rclpy.spin(node)
    rclpy.shutdown()
