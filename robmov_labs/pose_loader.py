#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

class PoseLoader(Node):
    def __init__(self ):
        super().__init__('pose_loader')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_poses)
        self.publisher = self.create_publisher(PoseArray, 'goal_list', 10)
        self.poses = self.read_file()
        self.flag = True
        
    def publish_poses(self):
        if self.publisher.get_subscription_count() > 0 and self.flag:
            self.flag = False
            msg = PoseArray()
            msg.poses = self.poses
            self.publisher.publish(msg)
            self.get_logger().info('Published poses')
    
    def read_file(self):
        poses = []
        import os
        print(os.getcwd())
        with open( 'src/robmov_labs/text/' + input("Ingrese el nombre del archivo de texto: ") + '.txt', 'r') as file:
            for line in file:
                x, y, theta = line.split(',')
                
                if "\n" in theta:
                    theta = theta.replace("\n", "")
                    
                if "pi" in theta:
                    theta = theta.replace("pi", "1")
                    theta = eval(theta) * np.pi
                
                q = quaternion_from_euler(0, 0, theta)
                pose = Pose(position = Point(x=float(x), y=float(y), z=float(0)), orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
                poses.append(pose)
                
        return poses
    
def main(args=None):
    rclpy.init(args=args)
    node = PoseLoader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()