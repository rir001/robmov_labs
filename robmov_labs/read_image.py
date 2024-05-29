#!/usr/bin/env python3
import cv2
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import matplotlib.pyplot as plt


VEL = 1.0


class ReadImage( Node ):

    def __init__( self ):
        super().__init__( 'image_reader' )
        self.bg = CvBridge()
        self.kinect_image_sub   = self.create_subscription( Image, '/kinect/image_raw', self.image_reader, 1 )
        self.position_pub       = self.create_publisher( Twist, '/commands/velocity', 1 )

        self.get_logger().info( "image_reader: ON" )

        self.speed = Twist()

        self.mask = None
        self.ready = 0
        self.mask_b = np.array([103, 106,  84])
        self.mask_t = np.array([126, 255, 255])
        self.n = 20


    def image_reader(self, image):
        cv_image = self.bg.imgmsg_to_cv2(image, desired_encoding='passthrough')
        self.cv_image = np.array(cv_image)

        if not self.mask:
        # if 0:
            self.thread_filter_selector()
        elif self.ready:
            image_ = self.filter_image()
            cv2.imshow("sas", image_[:, ::-1])
            cv2.waitKey(1)

            t, b, l, r = self.get_margin(image_)
            # plt.imsave(f"sas.png", self.cv_image[t:b, l:r])

            x = (l + r)/2
            area = ((t + b)/2) * ((l + r)/2)

            self.get_logger().info( f'pose: {x}' )

            if [t, b, l, r] == [0, image_.shape[0]-1, 0, image_.shape[1]-1]:
                self.set_velocity_angle(1.0)
                self.set_velocity_desp(0.0)
            elif   x < 280:
                self.set_velocity_angle( 1.2)
                self.set_velocity_desp(0.0)
            elif x > 360 :
                self.set_velocity_angle(-1.2)
                self.set_velocity_desp(0.0)
            else:
                self.set_velocity_angle(0.0)
                self.set_velocity_desp(0.2 * (1.5 - (area / 300_000)))


    def set_velocity_angle(self, vel):
        self.speed.angular.z = min(vel, VEL) if vel > 0 else max(vel, -VEL)
        self.position_pub.publish(self.speed)

    def set_velocity_desp(self, vel):
        self.speed.linear.x = min(vel, VEL) if vel > 0 else max(vel, -VEL)
        self.position_pub.publish(self.speed)

    def thread_filter_selector(self):
        self.mask = 1
        thread = Thread(target=self.filter_selector, daemon=True)
        thread.start()


    def filter_selector(self):
        windowName = 'HSV'
        cv2.namedWindow(windowName)
        cv2.createTrackbar('H_b'   , windowName, self.mask_b[0], 179, lambda x: None)
        cv2.createTrackbar('H_t'   , windowName, self.mask_t[0], 179, lambda x: None)
        cv2.createTrackbar('S_b'   , windowName, self.mask_b[1], 255, lambda x: None)
        cv2.createTrackbar('S_t'   , windowName, self.mask_t[1], 255, lambda x: None)
        cv2.createTrackbar('V_b'   , windowName, self.mask_b[2], 255, lambda x: None)
        cv2.createTrackbar('V_t'   , windowName, self.mask_t[2], 255, lambda x: None)

        cv2.createTrackbar('mask'  , windowName, self.n,  20, lambda x: None)
        cv2.setTrackbarMin('mask'  , windowName, 1)

        out = self.cv_image.copy()

        while True:
            if (cv2.waitKey(1) & 0xFF) == 120: break

            cv2.imshow(windowName, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))

            self.mask_b = np.array([
                cv2.getTrackbarPos('H_b', windowName),
                cv2.getTrackbarPos('S_b', windowName),
                cv2.getTrackbarPos('V_b', windowName)])
            self.mask_t = np.array([
                cv2.getTrackbarPos('H_t', windowName),
                cv2.getTrackbarPos('S_t', windowName),
                cv2.getTrackbarPos('V_t', windowName)])
            self.n = cv2.getTrackbarPos('mask', windowName)

            out = self.cv_image.copy()
            out = cv2.bitwise_and(out ,out, mask=cv2.inRange(
                cv2.cvtColor(out, cv2.COLOR_RGB2HSV),
                self.mask_b,
                self.mask_t))
            out = cv2.morphologyEx(out, cv2.MORPH_OPEN, np.array([[1]*self.n]*self.n))

        cv2.destroyAllWindows()
        self.ready = 1


    def filter_image(self):
        # plt.imsave(f"sus.png", self.cv_image)

        out = self.cv_image.copy()
        out = cv2.bitwise_and(out ,out, mask=cv2.inRange(
            cv2.cvtColor(out, cv2.COLOR_RGB2HSV),
            self.mask_b,
            self.mask_t))
        return cv2.morphologyEx(out, cv2.MORPH_OPEN, np.array([[1]*self.n]*self.n))


    def get_margin(self, image):
        step = 3

        t, b, l, r = 0, image.shape[0]-1, 0, image.shape[1]-1
        for y in range(0, image.shape[0], step):
            if np.sum(image[y, :]) > 0 and t == 0:
                t = y
            if np.sum(image[image.shape[0] - y - 1, :]) > 0 and b == image.shape[0]-1:
                b = image.shape[0] - y

        for x in range(0, image.shape[1], step):
            if np.sum(image[:, x]) > 0 and l == 0:
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
