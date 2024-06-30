#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import LaserScan


VEL = 0.4
WINDOW = 1
M1 = 0.1    # Margen 1, 5.7°
M2 = 0.2    # Margen 2, 11.5°


def get_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return np.arctan2(y2-y1, x2-x1)

def get_aprox_image_angles(lista):
    angles = []
    for i in range(len(lista)-WINDOW):
        angle = get_angle(lista[i], lista[i+WINDOW])
        if angle < 0: angle += 2*np.pi
        if angle > 2*np.pi: angle -= 2*np.pi
        angles.append(round(angle, 1))
    return angles

def find_more_common_angle(lista):
    return max(set(lista), key=lista.count)

def get_precise_image_angle(lista, aprox_angle):
    angles = []
    for i in range(len(lista)-WINDOW):
        angle = get_angle(lista[i], lista[i+WINDOW])
        if angle < 0: angle += 2*np.pi
        if angle > 2*np.pi: angle -= 2*np.pi
        if round(angle, 1) == aprox_angle: angles.append(angle)
    
    precise_angle = np.round(np.mean(angles),3)
    return precise_angle



class Lab3_Nav( Node ):

    def __init__( self ):
        super().__init__( 'movimiento_lab3' )

        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )
        self.lidar_sub                  = self.create_subscription( LaserScan, '/scan', self.lidar_cb, 1 )
        self.particles_filter_sub       = self.create_subscription( PoseArray, '/particles', self.pf_cb, 1 )
        self.particles_filter_pub       = self.create_publisher( PoseArray, '/moved_particles', 1 )

        self.speed = Twist()

        self.angles = []
        self.prev_selected_angle = 0
        self.x = 320.0
        self.rango = np.arange(-np.pi/2, np.pi/2, step=0.01745329238474369)


    
    def lidar_cb(self, image):
        self.get_logger().info(f'image.ranges: {list(image.ranges)}')
        muestra = list(image.ranges)
        points = [(0, 0)]
        front_points = []
        
        for n in range(62, 119):
            if 81 <= n <= 100:
                front_points.append(muestra[n])
            if muestra[n] == 4.0: continue

            x = muestra[n]*np.cos(self.rango[n])
            y = muestra[n]*np.sin(self.rango[n])

            points.append((x, y))
        
        self.distance_from_wall = np.mean(front_points)


        h = get_aprox_image_angles(points)
        aprox_angle = find_more_common_angle(h)
        precise_angle = get_precise_image_angle(points, aprox_angle)

        if len(self.angles) >= 10:
            self.angles.pop(0)
        self.angles.append(precise_angle)

        # self.get_logger().info(f'angle: {precise_angle}')


    def pf_cb(self, msg):
        '''
        Se determina el ángulo más probable y, en caso de que sea similar al anterior, se avanza.
        En caso contrario, se gira hasta estar perpendicular a una pared.

        Si es que se decide avanzar se chequea la distancia a la pared, si es menor a X se gira 90° a la derecha.
        Sino, se avanza.
        '''
        return 
        proposed_angle = find_more_common_angle(self.angles)
        picked_angles = []
        for angle in self.angles:
            if proposed_angle - M1 <= angle <= proposed_angle + M1:
                picked_angles.append(angle)
            if proposed_angle - M1 <= angle + 2*np.pi <= proposed_angle + M1:
                picked_angles.append(angle + 2*np.pi)
            if proposed_angle - M1 <= angle - 2*np.pi <= proposed_angle + M1:
                picked_angles.append(angle - 2*np.pi)
            
        if len(picked_angles) > 1:
            selected_angle = np.mean(picked_angles)
        else:
            selected_angle = self.angles[-1]
        
        if (self.prev_selected_angle - M2 <= selected_angle <= self.prev_selected_angle + M2 or
            self.prev_selected_angle - M2 <= selected_angle + 2*np.pi <= self.prev_selected_angle + M2 or
            self.prev_selected_angle - M2 <= selected_angle - 2*np.pi <= self.prev_selected_angle + M2):
            #  Verificamos si hay una pared cerca, 
            # si es así, giramos 90° a la derecha, sino avanzamos
            pass
        else:
            # Rotamos hasta estar perpendicular a la pared
            pass

        self.prev_selected_angle = selected_angle
        
        # Publicar el movimiento a realizar
        self.cmd_vel_mux_pub.publish(self.speed)

        # IMPORTANTE
        # De alguna forma verificar que ya se haya movido el robot antes de publicar las nuevas particulas
        # (para asegurarse de que se lea la nueva pose en el momento adecuado)
        # IMPORTANTE

        # Publicar las nuevas particulas (aplicando el movimiento del robot a las particuals anteriores)
        new_particles = PoseArray()
        new_particles.header = msg.header
    

        
    


            
        

            
                
            


        


def main():
    rclpy.init()
    node = Lab3_Nav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()


