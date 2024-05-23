#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import matplotlib.pyplot as plt



class ReadImage( Node ):

    def __init__( self ):
        super().__init__( 'image_reader' )
        self.kinect_image_sub = self.create_subscription( Image, '/kinect/image_raw', self.image_reader, 10 )
        self.get_logger().info( "sos" )


    def image_reader(self, image):
        # self.get_logger().info( image.data )

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        plt.imsave("sas.png", cv_image)


def main(args=None):
    rclpy.init(args=args)
    node = ReadImage()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
