#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
from threading import Thread


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__( 'obstacle_detector' )
        self.subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.publisher = self.create_publisher(Vector3, 'occupancy_state', 10)
        self.bridge = CvBridge()
        self.vector = Vector3()
        self.data_cv = None

    def depth_cb( self, data ):
        self.data_cv = self.bridge.imgmsg_to_cv2( data )

        l = self.data_cv[::, :213].min()
        c = self.data_cv[::, 213:426].min()
        r = self.data_cv[::, 426:].min()
        
        self.vector.x = float(int(l < 0.7 or np.isnan(l))) 
        self.vector.y = float(int(c < 0.7 or np.isnan(c)))
        self.vector.z = float(int(r < 0.7 or np.isnan(r)))
        
        self.publisher.publish(self.vector)
        
    def mostrar(self):
        while True:
            if self.data_cv:
                cv2.imshow('frame1', self.data_cv) 
                
                if cv2.waitKey(1) & 0xFF == 27: 
                    break


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    thread_screen = Thread(target=node.mostrar)
    thread_screen.start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()