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

from time import sleep

import matplotlib.pyplot as plt


VEL = 0.2

TOLERANCIA_DESP = 0.000001
TOLERANCIA_ANG  = 0.0001

class DeadReckoningNav( Node ):

    def __init__( self ):
        super().__init__( 'dead_reckoning_nav' )

        #              desp / angle
        self.velocity = [[0], [0]]
        self.setpoint = [[0], [0]]
        self.state    = [[0], [0]]


        self.real_pose_sub              = self.create_subscription( Pose, '/real_pose', self.real_pose_loader, 1 )
        self.goal_list_sub              = self.create_subscription( PoseArray, 'goal_list', self.accion_mover_cb_thread, 10 )

        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )

        # angle PID
        self.velocity_angle_sub         = self.create_subscription( Float64, 'velocity_angle', self.velocity_angle_sub_thread, 1 )
        self.setpoint_angle_pub         = self.create_publisher( Float64, 'setpoint_angle', 1 )
        self.state_angle_pub            = self.create_publisher( Float64, 'state_angle', 1 )

        # displacement PID
        self.velocity_desp_sub          = self.create_subscription( Float64, 'velocity_desp', self.velocity_desp_sub_thread, 1 )
        self.setpoint_desp_pub          = self.create_publisher( Float64, 'setpoint_desp', 1 )
        self.state_desp_pub             = self.create_publisher( Float64, 'state_desp', 1 )


        self.speed = Twist()
        self.pause_robot = False

        self.x = 0.0
        self.y = 0.0
        self.w = 0.0


    def real_pose_thread(self, position):
        thread = Thread(target=self.real_pose_loader, args=(position,))
        thread.start()

    def accion_mover_cb_thread(self, goal_list):
        thread = Thread(target=self.accion_mover_cb, args=(goal_list,))
        thread.start()

    def velocity_desp_sub_thread(self, vel):
        thread = Thread(target=self.set_velocity_desp, args=(vel,))
        thread.start()

    def set_velocity_desp(self, vel):
        self.speed.linear.x = min(vel.data, VEL) if vel.data > 0 else max(vel.data, -VEL)
        self.state[0].append(self.state[0][-1])
        self.setpoint[0].append(self.setpoint[0][-1])
        self.velocity[0].append(self.speed.linear.x)
        self.cmd_vel_mux_pub.publish(self.speed)

    def velocity_angle_sub_thread(self, vel):
        thread = Thread(target=self.set_velocity_angle, args=(vel,))
        thread.start()

    def set_velocity_angle(self, vel):
        self.speed.angular.z = min(vel.data, VEL) if vel.data > 0 else max(vel.data, -VEL)
        self.state[1].append(self.state[1][-1])
        self.setpoint[1].append(self.setpoint[1][-1])
        self.velocity[1].append(self.speed.angular.z)
        self.cmd_vel_mux_pub.publish(self.speed)

    def send_setpoint_desp(self, data):
        msg = Float64()
        msg.data = data
        self.state[0].append(self.state[0][-1])
        self.setpoint[0].append(data)
        self.velocity[0].append(self.speed.linear.x)
        self.setpoint_desp_pub.publish( msg )
        # self.get_logger().info( 'send displacement target: %.4f' % ( data ) )

    def send_setpoint_angle(self, data):
        msg = Float64()
        msg.data = data
        self.state[1].append(self.state[1][-1])
        self.setpoint[1].append(data)
        self.velocity[1].append(self.speed.angular.z)
        self.setpoint_angle_pub.publish( msg )
        # self.get_logger().info( 'send angle target: %.4f' % ( data ) )

    def send_state_desp(self, data):
        msg = Float64()
        msg.data = data
        self.state[0].append(data)
        self.setpoint[0].append(self.setpoint[0][-1])
        self.velocity[0].append(self.speed.linear.x)
        self.state_desp_pub.publish( msg )
        # self.get_logger().info( 'send actual displacement: %.4f' % ( data ) )

    def send_state_angle(self, data):
        msg = Float64()
        msg.data = data
        self.state[1].append(data)
        self.setpoint[1].append(self.setpoint[1][-1])
        self.velocity[1].append(self.speed.angular.z)
        self.state_angle_pub.publish( msg )
        # self.get_logger().info( 'send actual angle: %.4f' % ( data ) )

    def real_pose_loader(self, position):
        self.x = position.position.x
        self.y = position.position.y

        roll, pitch, self.w = euler_from_quaternion((
            position.orientation.x,
            position.orientation.y,
            position.orientation.z,
            position.orientation.w
            )
        )

    def accion_mover_cb(self, goal_list):
        for pose in goal_list.poses:

            angles = euler_from_quaternion([pose.orientation.x,  pose.orientation.y, pose.orientation.z, pose.orientation.w])
            goal = [pose.position.x, pose.position.y, angles[2]]

            self.get_logger().info(f'Next Goal: {goal}')
            self.mover_robot_a_destino(goal)

        self.get_logger().info(f'Finish')

        # self.velocity = [[], []]
        # self.setpoint = [[], []]
        # self.state    = [[], []]

        plt.figure(1)
        plt.title("Desplazamiento")
        plt.plot(np.arange(0, len(self.velocity[0])), self.velocity[0], color="r", label="desp velocity")
        plt.plot(np.arange(0, len(self.setpoint[0])), self.setpoint[0], color="g", label="desp setpoint")
        plt.plot(np.arange(0, len(self.state[0]))   , self.state[0]   , color="b", label="desp state")

        plt.figure(2)
        plt.title("Angulo")
        plt.plot(np.arange(0, len(self.velocity[1])), self.velocity[1], color="r", label="ang velocity")
        plt.plot(np.arange(0, len(self.setpoint[1])), self.setpoint[1], color="g", label="ang setpoint")
        plt.plot(np.arange(0, len(self.state[1]))   , self.state[1]   , color="b", label="ang state")

        plt.show()


    def mover_robot_a_destino(self, goal_pose):
        x, y, theta = goal_pose
        speed_command_list = []

        if x != 0:
            if x < 0:
                speed_command_list.append((0, pi))
                y = -y
            speed_command_list.append((x, 0))

        if y != 0:
            if y < 0:
                speed_command_list.append((0, pi*3/2))
                speed_command_list.append((-y, 0))
            else:
                speed_command_list.append((0, pi/2))
                speed_command_list.append((y, 0))

        if len(speed_command_list) > 0:
            theta_actual = sum(np.array(speed_command_list)[:, 1])
        else:
            theta_actual = 0

        if round(theta_actual,2) != round(theta,2):
            if theta_actual < theta:
                speed_command_list.append((0, theta - theta_actual))
            else:
                speed_command_list.append((0, 2*pi - theta_actual + theta))

        self.get_logger().info(str(speed_command_list))

        self.aplicar_velocidad(speed_command_list)


    def aplicar_velocidad( self, speed_command_list):
        for command in speed_command_list:

            if command[0] != 0:
                start_x = self.x
                start_y = self.y
                self.send_setpoint_desp(command[0])

                pose = np.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
                while abs(self.speed.linear.x) > TOLERANCIA_DESP or self.speed.linear.x == 0:
                    self.send_state_desp(pose)
                    pose = np.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)

                self.send_setpoint_desp(0.0)
                self.send_state_desp(0.0)
                sleep(0.1)

            if command[1] != 0:
                start_w = self.w
                self.send_setpoint_angle(command[1])

                pose = dif_calulator(self.w, start_w)
                while abs(self.speed.angular.z) > TOLERANCIA_ANG or self.speed.angular.z == 0:
                    self.send_state_angle(pose)
                    pose = dif_calulator(self.w, start_w)

                self.send_setpoint_angle(0.0)
                self.send_state_angle(0.0)
                sleep(0.1)



def dif_calulator(ang1, ang2):
    ang = ang1 - ang2
    return ang if ang > -pi else ang +  2*pi




def main():
    rclpy.init()
    node = DeadReckoningNav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
