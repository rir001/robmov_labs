#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt

class RealPosReader(Node):
    def __init__(self):
        super().__init__('real_pos_reader')
        self.pos_subscription = self.create_subscription(Odometry, '/real_pose', self.real_pose_cb, 10)
        self.prev_pos = None
        self.road = []
        self.contador = 0
        self.flag = False

    def real_pose_cb(self, pos):
        print("b")
        x = pos.pose.pose.position.x
        y = pos.pose.pose.position.y
        z = pos.pose.pose.position.z
        orientation = pos.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        
        if pos != self.prev_pos:
            self.flag = True
            self.get_logger().info('Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw))
            self.road.append((x, y))
            self.prev_pos = pos
            self.contador = 0
        else:
            self.contador += 1

        if self.contador == 100 and self.flag:
            self.plot_road()
        
    def plot_road(self):
        x = [pos[0] for pos in self.road]
        y = [pos[1] for pos in self.road]
        fig, ax = plt.subplots()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.plot(x, y)
        plt.show()
        plt.save('pos_real.png')

def main():
    rclpy.init()
    pos_reader = RealPosReader()
    rclpy.spin(pos_reader)

if __name__ == '__main__':
    main()