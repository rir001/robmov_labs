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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

import cv2

VEL = 0.4

class PasilloNav( Node ):

    def __init__( self ):
        super().__init__( 'navegacion_pasillo' )

        self.camera_sub                 = self.create_subscription( Image, '/camera/depth/image_raw', self.show_camera, 1 )
        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )

        # angle PID
        self.velocity_angle_sub         = self.create_subscription( Float64, 'velocity', self.velocity_angle_sub_thread, 1 )
        self.setpoint_angle_pub         = self.create_publisher( Float64, 'setpoint', 1 )
        self.state_angle_pub            = self.create_publisher( Float64, 'state', 1 )

        self.speed = Twist()
        self.bg = CvBridge()

        self.send_setpoint_angle(320.0)



    def velocity_angle_sub_thread(self, vel):
        thread = Thread(target=self.set_velocity_angle, args=(vel,))
        thread.start()

    def set_velocity_angle(self, vel):
        self.speed.angular.z = min(vel.data, VEL) if vel.data > 0 else max(vel.data, -VEL)
        self.cmd_vel_mux_pub.publish(self.speed)

    def send_setpoint_angle(self, data):
        msg = Float64()
        msg.data = data
        self.setpoint_angle_pub.publish( msg )

    def send_state_angle(self, data):
        msg = Float64()
        msg.data = data
        self.state_angle_pub.publish( msg )

    def set_velocity_desp(self, vel):
        self.speed.linear.x = min(vel, VEL) if vel > 0 else max(vel, -VEL)
        self.cmd_vel_mux_pub.publish(self.speed)

    def show_camera(self, image):
        cv_image = self.bg.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_image = np.nan_to_num(np.array(cv_image))

        line = cv_image[200, :]

        x = np.sum(line * (line > np.max(line)/3) * np.arange(1, 641)) / (640)
        x = min(639.0, max(0.0, x))



        if   line[416] < 0.5:
            self.set_velocity_desp(0.0)
            self.send_state_angle(213.0)
        elif line[213] < 0.5:
            self.set_velocity_desp(0.0)
            self.send_state_angle(426.0)
        elif line[int(x)] < 0.5:
            self.set_velocity_desp(0.0)
            self.send_state_angle(x)
        else:
            self.set_velocity_desp(0.2)
            self.send_state_angle(x)



        cv_image[:, int(x)] = 0
        cv2.imshow("camera", cv_image)
        cv2.waitKey(1)



def main():
    rclpy.init()
    node = PasilloNav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
