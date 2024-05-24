#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import matplotlib.pyplot as plt


ESCALA = 130

class WriterPose( Node ):

    def __init__( self ):
        super().__init__( 'pose_writer_nav' )

        self.real_pose_sub = self.create_subscription( Pose, '/real_pose', self.real_pose_writer, 1 )

        self.canvas = np.ones((500, 500))


    def real_pose_writer(self, position):
        x = position.position.x * ESCALA
        y = position.position.y * ESCALA

        self.get_logger().info(f'x:{x} y:{y}')

        self.canvas[500 - int(y), int(x)] = 0

        plt.imsave("name.png", self.canvas)


def main():
    rclpy.init()
    node = WriterPose()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
