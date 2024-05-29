#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

from time import sleep


class PoseLoader(Node):

    def __init__(self ):
        super().__init__('pose_loader')
        self.poses = self.read_file()
        self.publisher = self.create_publisher(PoseArray, 'goal_list', 10)
        sleep(2)
        self.timer = self.create_timer(0.1, self.publish_poses)
        self.flag = True


    def publish_poses(self):
        if self.publisher.get_subscription_count() > 0 and self.flag:
            self.get_logger().info('Published poses')
            msg = PoseArray()
            msg.poses = self.poses
            self.publisher.publish(msg)
            self.get_logger().info('Published poses')
            self.flag = False


    def read_file(self):
        poses = []
        name = 'cuadra2'
        with open(f'src/robmov_labs/text/{name}.txt', 'r') as file:
            for line in file:
                x, y, theta = line.strip().replace("pi", str(np.pi)).split(',')
                q = quaternion_from_euler(0, 0, eval(theta))
                poses.append(Pose(
                    position = Point(x=float(x), y=float(y), z=float(0)),
                    orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) #testear args o kwargs
                    )
                )
        return poses


def main(args=None):
    rclpy.init(args=args)
    node = PoseLoader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
