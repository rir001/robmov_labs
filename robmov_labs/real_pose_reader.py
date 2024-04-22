#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class RealPoseReader( Node ):

    def __init__( self ):
        super().__init__( 'real_pose_node' )
        self.odom_sub = self.create_subscription(Pose, '/real_pose', self.real_pose_cb, 10 )
        self.prev = None
        self.road = []
        self.contador = 0
        self.flag1 = False
        self.flag2 = False
        self.flag3 = False

    def real_pose_cb( self, pos ):
        if not self.flag3:
            x = pos.position.x
            y = pos.position.y
            z = pos.position.z
            roll, pitch, yaw = euler_from_quaternion( ( pos.orientation.x,
                                                        pos.orientation.y,
                                                        pos.orientation.z,
                                                        pos.orientation.w ) )
            pos = (round(x,4), round(y,4), round(yaw,4))
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
        # plt.savefig('imgs/real_pose_corrected.png')
        # plt.savefig('imgs/real_pose_not_corrected.png')
        self.flag3 = True


def main():
    rclpy.init()
    real_pose_reader = RealPoseReader()
    rclpy.spin( real_pose_reader )

if __name__ == '__main__':
    main()