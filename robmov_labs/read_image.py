#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import matplotlib.pyplot as plt

import cv2
import numpy as np


class ReadImage( Node ):

    def __init__( self ):
        super().__init__( 'image_reader' )
        self.kinect_image_sub = self.create_subscription( Image, '/kinect/image_raw', self.image_reader, 10 )
        self.get_logger().info( "sos" )
        self.n = 0


    def image_reader(self, image):
        # self.get_logger().info( image.data )

        self.n += 1

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        cv_image = np.array(cv_image)

        t, b, l, r = get_margin(filter_image(cv_image))
        print(self.n, t, b, l, r)
        plt.imsave(f"{self.n}sas.png", cv_image[t:b, l:r])

        # plt.imsave(f"{self.n}sas.png", cv_image)
        print(self.n)


def filter_image(rgb_image):
    image_hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

    mask_H = (image_hsv[:, :, 0] <= 130) * (image_hsv[:, :, 0] >= 100)
    mask_S = (image_hsv[:, :, 1] <= 300) * (image_hsv[:, :, 1] >= 100)
    mask_V = (image_hsv[:, :, 2] <= 250) * (image_hsv[:, :, 2] >= 150)

    out = rgb_image.copy()
    out[:, :, 0] = out[:, :, 0] * mask_H * mask_S * mask_V
    out[:, :, 1] = out[:, :, 1] * mask_H * mask_S * mask_V
    out[:, :, 2] = out[:, :, 2] * mask_H * mask_S * mask_V

    plt.imsave(f"sos.png", out)

    kernel = np.array([[1]*7]*7)


    return cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)


def get_margin(image):
    step = 3
    t, b, l, r = 0, image.shape[0]-1, 0, image.shape[1]-1
    for y in range(0, image.shape[0], step):
        if np.sum(image[y, :]) > 0 and not t:
            t = y
        if np.sum(image[image.shape[0] - y - 1, :]) > 0 and b == image.shape[0]-1:
            b = y

    for x in range(0, image.shape[1], step):
        if np.sum(image[:, x]) > 0 and not l:
            l = x
        if np.sum(image[:, image.shape[1] - x - 1]) > 0 and r == image.shape[1]-1:
            r = image.shape[1] - x

    return t, b, l, r

def main(args=None):
    rclpy.init(args=args)
    node = ReadImage()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
