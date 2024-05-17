#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, Quaternion, Vector3


OMEGA = 1.0          # rad/s
T_AJUSTE = 1.1085    # factor de ajuste para el tiempo de duracion de los movimientos
VEL = 0.2            # m/s

class DeadReckoningNav( Node ):

    def __init__( self ):
        super().__init__( 'dead_reckoning_nav' )

        self.occupancy_state_sub        = self.create_subscription( Vector3, 'occupancy_state', self.trayectoria_cb, 10 )
        self.real_pose_sub              = self.create_subscription( Pose, '/real_pose', self.real_pose_loader, 1 )
        self.goal_list_sub              = self.create_subscription( PoseArray, 'goal_list', self.accion_mover_cb_thread, 10 )

        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )

        # angle PID
        self.velocity_angle_sub         = self.create_subscription( Vector3, 'velocity_angle', self.velocity_angle_sub_thread, 1 )
        self.setpoint_angle_pub         = self.create_publisher( Float64, 'setpoint_angle', 1 )
        self.state_angle_pub            = self.create_publisher( Float64, 'state_angle', 1 )

        # displacement PID
        self.velocity_desp_sub          = self.create_subscription( Vector3, 'velocity_desp', self.velocity_desp_sub_thread, 1 )
        self.setpoint_desp_pub          = self.create_publisher( Float64, 'setpoint_desp', 1 )
        self.state_desp_pub             = self.create_publisher( Float64, 'state_desp', 1 )

        self.speed = Twist()
        self.timer_period = 0.2 # seconds
        self.pause_robot = False

        self.x = 0
        self.y = 0
        self.w = 0


    def real_pose_thread(self, position):
        thread = Thread(target=self.real_pose_loader, args=(position,))
        thread.start()

    def accion_mover_cb_thread(self, goal_list):
        thread = Thread(target=self.accion_mover_cb, args=(goal_list,))
        thread.start()

    def velocity_angle_sub_thread(self, vel):
        pass

    def velocity_desp_sub_thread(self, vel):
        pass

    def send_setpoint_desp(self, data):
        msg = Float64()
        msg.data = data
        self.setpoint_desp_pub.publish( msg )
        self.get_logger().info( 'send displacement target: %.4f' % ( data ) )

    def send_setpoint_angle(self, data):
        msg = Float64()
        msg.data = data
        self.setpoint_angle_pub.publish( msg )
        self.get_logger().info( 'send angle target: %.4f' % ( data ) )

    def send_state_desp(self, data):
        msg = Float64()
        msg.data = data
        self.state_desp_pub.publish( msg )
        self.get_logger().info( 'send actual displacement: %.4f' % ( data ) )

    def send_state_angle(self, data):
        msg = Float64()
        msg.data = data
        self.state_angle_pub.publish( msg )
        self.get_logger().info( 'send actual angle: %.4f' % ( data ) )

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

        theta_actual = sum(np.array(speed_command_list)[:, 1])

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
                self.get_logger().info("sas")
                self.get_logger().info(str(np.float64(command[0])))
                self.setpoint_angle_pub.publish(np.float64(command[0]))

                # while True:
                #     self.get_logger().info(str(self.x))
                #     self.get_logger().info(str(self.y))



            # self.speed.linear.x = command[0]
            # self.speed.angular.z = command[1]

            # contador = 0


            # delta = 0
            # while t_actual - t_inicial - delta < t_ejecucion:

            #     pause = True
            #     while self.pause_robot:
            #         if pause:
            #             self.get_logger().info("Robot paused")
            #             p_start = self.get_clock().now().nanoseconds
            #         pause = False
            #     if not pause:
            #         delta += self.get_clock().now().nanoseconds - p_start


            #     if (t_actual - t_inicial - delta) > self.timer_period*contador*10**9:
            #         if pause:
            #             contador += 1
            #         self.cmd_vel_mux_pub.publish(self.speed)
            #         self.get_logger().info(f'Moving: v={self.speed.linear.x}, w={self.speed.angular.z},     {contador}')

            #     t_actual = self.get_clock().now().nanoseconds


    def trayectoria_cb(self, vector):
        lcr = (vector.x, vector.y, vector.z)
        if lcr == (0.0, 0.0, 0.0):
            self.pause_robot = False
        else:
            self.pause_robot = True
            if int(lcr[0]): self.get_logger().info("obstacle left")
            if int(lcr[1]): self.get_logger().info("obstacle center")
            if int(lcr[2]): self.get_logger().info("obstacle right")







def main():
    rclpy.init()
    node = DeadReckoningNav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
