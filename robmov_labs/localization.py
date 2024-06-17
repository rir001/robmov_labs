#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

import cv2

VEL = 0.4
PLOT = 1

class PasilloNav( Node ):

    def __init__( self ):
        super().__init__( 'navegacion_pasillo' )

        self.ploted = False

        self.lidar_sub                  = self.create_subscription( LaserScan, '/scan',                 self.proces_lidar,  1 )
        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )

        self.bg = CvBridge()


    def show_camera(self, image):
        self.get_logger().info( "Camera" )
        cv_camera = self.bg.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_camera = np.nan_to_num(np.array(cv_camera))

        cv2.imshow("camera", cv_camera)
        cv2.waitKey(1)


    def load_plot(self, rango):
        plt.ion()
        self.fig = plt.figure()
        self.fig.canvas.set_window_title('Lidar')
        ax = self.fig.add_subplot(111, polar=True)
        self.line, = ax.plot(rango, np.ones(len(rango)), color='b', lw=3)
        ax.set_theta_zero_location("N")
        ax.set_rmax(2.5)
        ax.set_rlabel_position(-22.5)
        ax.grid(True)
        self.ploted = True


    def show_lidar(self, image):
        if PLOT:
            if not self.ploted: self.load_plot(np.arange(image.angle_min, image.angle_max, image.angle_increment))
            self.line.set_ydata(image.ranges)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.get_logger().info( f"{ image.ranges }" )

    def proces_lidar(self, image):
        self.show_lidar(image)



def main():
    rclpy.init()
    node = PasilloNav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
