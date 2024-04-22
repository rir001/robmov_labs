#!/usr/bin/env python3
import numpy as np
np.float = float

import rclpy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, Quaternion, Vector3
from threading import Thread


OMEGA = 1.0          # rad/s
T_AJUSTE = 1.1085    # factor de ajuste para el tiempo de duracion de los movimientos
VEL = 0.2            # m/s

class DeadReckoningNav( Node ):
    def __init__( self ):
        super().__init__( 'dead_reckoning_nav' )
        self.subscription = self.create_subscription(PoseArray, 'goal_list', self.thread_function, 10)
        self.subscription2 = self.create_subscription(Vector3, 'occupancy_state', self.trayectoria_cb, 10)
        self.cmd_vel_mux_pub = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)
        self.speed = Twist()
        self.timer_period = 0.2 # seconds
        self.pause_robot = False
        
        # Tests
        # self.aplicar_velocidad( [ [0.5, 0.0, 3.0], [0.0, 0.2, 1.0], [0.5, 0.0, 3.0], [0.0, 0.2, 1.0], [0.0, 0.0, 1.0] ] )
        
        # self.mover_robot_a_destino( [1.0, 1.0, np.pi] )
        
        # lista = PoseArray()
        # q1 = quaternion_from_euler(0,0,np.pi/2)
        # pose1 = Pose(position = Point(x=1.0, y=1.0, z=0.0), orientation = Quaternion(x = q1[0], y = q1[1], z = q1[2], w = q1[3]))
        # q2 = quaternion_from_euler(0,0,np.pi)
        # pose2 = Pose(position=Point(x=-1.0, y=1.0, z=0.0), orientation=Quaternion(x = q2[0], y=q2[1], z=q2[2], w=q2[3]))
        # q3 = quaternion_from_euler(0,0,5*np.pi/4)
        # pose3 = Pose(position=Point(x=0.0, y=2.0, z=0.0), orientation=Quaternion(x = q3[0], y = q3[1], z = q3[2], w = q3[3]))
        # lista.poses = [pose1, pose2, pose3]
        # self.accion_mover_cb(lista)


    def aplicar_velocidad( self, speed_command_list ):
        for command in speed_command_list: 

            self.speed.linear.x = command[0]
            self.speed.angular.z = command[1]
            t_ejecucion = command[2]*10**9

            t_inicial = self.get_clock().now().nanoseconds
            t_actual = self.get_clock().now().nanoseconds
            contador = 0

            
            delta = 0

            while t_actual - t_inicial - delta < t_ejecucion:
                
                pause = 1
                while self.pause_robot:
                    if pause:
                        self.get_logger().info("Robot paused")
                        p_start = self.get_clock().now().nanoseconds
                    pause = 0
                delta += self.get_clock().now().nanoseconds - p_start


                if (t_actual - t_inicial - delta) > self.timer_period*contador*10**9:
                    contador += 1
                    self.cmd_vel_mux_pub.publish(self.speed)
                    self.get_logger().info(f'Moving: v={self.speed.linear.x}, w={self.speed.angular.z},     {contador}')
                    
                t_actual = self.get_clock().now().nanoseconds
            

    def mover_robot_a_destino(self, goal_pose ):
        x, y, theta = goal_pose
        speed_command_list = []
        
        theta_actual = 0
        
        if x != 0:
            if x < 0:
                speed_command_list.append(self.gira(np.pi))
                theta_actual += np.pi
                y = -y
        
            speed_command_list.append(self.avanza(np.abs(x)))
        
        if y != 0:
            if y < 0:
                speed_command_list.append(self.gira(3*np.pi/2))
                theta_actual += 3*np.pi/2
            else:
                speed_command_list.append(self.gira(np.pi/2))
                theta_actual += np.pi/2

            speed_command_list.append(self.avanza(np.abs(y)))
        
        
        if round(theta_actual,2) != round(theta,2):
            if theta_actual < theta:
                speed_command_list.append(self.gira( theta - theta_actual ))
            else:
                speed_command_list.append(self.gira( 2*np.pi - theta_actual + theta ))

        self.aplicar_velocidad(speed_command_list)
    
    
    def accion_mover_cb(self, goal_list):
        for pose in goal_list.poses:
            
            angles = euler_from_quaternion([pose.orientation.x,  pose.orientation.y, pose.orientation.z, pose.orientation.w])
            goal = [pose.position.x, pose.position.y, angles[2]]

            self.get_logger().info(f'Next Goal: {goal}')
            self.mover_robot_a_destino(goal)
            
    
    def trayectoria_cb(self, vector):
        lcr = (vector.x, vector.y, vector.z)
        
        if lcr == (0.0, 0.0, 0.0):
            self.pause_robot = False
        
        else:
            self.pause_robot = True
            if int(lcr[0]):
                self.get_logger().info("obstacle left")
            if int(lcr[1]):
                self.get_logger().info("obstacle center")
            if int(lcr[2]):
                self.get_logger().info("obstacle right")

        
    def thread_function(self, goal_list):
        thread = Thread(target=self.accion_mover_cb, args=(goal_list,))
        thread.start()


############################
### Funciones Auxiliares ###
############################

def gira(self, angulo: float):
    '''
    En base a un angulo (en radianes), se retorna el comando de giro necesario para que el robot gire ese angulo.
    Retorna una lista con la velocidad lineal, velocidad angular y tiempo de duracion del movimiento [v, w, t].
    '''
    tiempo = float(angulo/OMEGA)*T_AJUSTE
    return [0.0, OMEGA, tiempo]
    
def avanza(self, distancia: float):
    '''
    En base a una distancia, se retorna el comando de avance necesario para que el robot avance esa distancia.
    Retorna una lista con la velocidad lineal, velocidad angular y tiempo de duracion del movimiento [v, w, t].
    '''
    tiempo = float(distancia/VEL)
    return [VEL, 0.0, tiempo]



def main():
    rclpy.init()
    node = DeadReckoningNav()
    rclpy.spin( node )

        
if __name__ == '__main__':
    main()
