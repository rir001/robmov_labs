#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class OdometryReader( Node ):

    def __init__( self ):
        super().__init__( 'odom_reader_node' )
        self.odom_sub = self.create_subscription( Odometry, '/odom', self.odometry_cb, 10 )
        self.prev = None
        self.road = []
        self.contador = 0
        self.flag1 = False
        self.flag2 = False
        self.flag3 = False

    def odometry_cb( self, odom ):
        if not self.flag3:
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            z = odom.pose.pose.position.z
            w = odom.pose.pose.position.w
        
            roll, pitch, yaw = euler_from_quaternion( ( x, y, z, w ) )

            pos = (round(x, 4), round(y, 4), round(yaw, 4))

            if pos != self.prev:
                self.contador = 0
                if self.flag1:
                    self.flag2 = True
                    # self.get_logger().info( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) ) 
                else:
                    self.flag1 = True
                
            self.prev = pos
            
            if self.flag2:    
                if self.contador == 20:
                    self.plot_road()
                    self.contador = 0
                
                else:
                    self.road.append((x, y))
                    self.contador += 1
                
            # print(self.contador, pos, self.prev, self.flag1, self.flag2) 
    
    def plot_road(self):
        x = [pos[0] for pos in self.road]
        y = [pos[1] for pos in self.road]
        fig, ax = plt.subplots()
        ax.set_xlim(-2, 4)
        ax.set_ylim(-2, 4)
        ax.plot(x, y)
        plt.show()
        # plt.savefig('imgs/corrected_odom.png')
        # plt.savefig('imgs/not_corrected_odom.png')
        self.flag3 = True


def main( args = None ):
    rclpy.init( args = args )
    odom_reader = OdometryReader()
    rclpy.spin( odom_reader )
    rclpy.shutdown()


if __name__ == '__main__':
    main()
