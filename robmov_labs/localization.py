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
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

import cv2
import random

VEL = 0.4
PLOT = 1
PARTICLES = 100

class Localization( Node ):

    def __init__( self ):
        super().__init__( 'localization' )

        self.ploted = False
        self.particles = self.initialize_particles( PARTICLES )

        self.lidar_sub          = self.create_subscription( LaserScan, '/scan', self.process_lidar,  1 )
        self.cmd_vel_mux_pub    = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )

        self.bg = CvBridge()


    def initialize_particles(self, num_particles):
        particles = []
        for _ in range(num_particles):
            particle = Pose()
            particle.position.x = random.uniform(0, 10)
            particle.position.y = random.uniform(0, 10)
            particle.orientation.w = 1.0
            particles.append(particle)
        return particles
    

    def show_camera(self, image):
        self.get_logger().info( "Camera" )
        cv_camera = self.bg.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_camera = np.nan_to_num(np.array(cv_camera))

        cv2.imshow("camera", cv_camera)
        cv2.waitKey(1)


    def load_plot(self, rango):
        plt.ion()
        self.fig = plt.figure()
        self.fig.canvas.set_window_title('Lidar')
        ax = self.fig.add_subplot(111, polar=True)
        self.line, = ax.plot(rango, np.ones(len(rango)), color='b', lw=3)
        ax.set_theta_zero_location("N")
        ax.set_rmax(2.5)
        ax.set_rlabel_position(-22.5)
        ax.grid(True)
        self.ploted = True


    def show_lidar(self, image):
        if not self.ploted: self.load_plot(np.arange(image.angle_min, image.angle_max, image.angle_increment))
        self.line.set_ydata(image.ranges)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.get_logger().info( f"{ image.ranges }" )

    def process_lidar(self, image):
        if PLOT: self.show_lidar(image)

        try:
            # Procesamos la imagen para obtner el mapa visto por el robot
            fixed_map, angle = correct_angle(image.ranges)
            # self.get_logger().info( f"Angle: {angle}" )
            # self.get_logger().info( f"Fixed Map: {fixed_map}" )
        except:
            self.get_logger().info("Error")

        # Luego, hacemos map matching para obtener la posición del robot en el mapa
        mapa = cv2.imread("maps/mapa.png", cv2.IMREAD_GRAYSCALE)

        # Finalmente, publicamos la posición del robot en el mapa

        # Meterle un contador, para que despues de x iteraciones, cree particulas nuevas (evitar errores)





def main():
    rclpy.init()
    node = Localization()
    rclpy.spin( node )


if __name__ == '__main__':
    main()


# UTILS

def correct_angle(muestra):
    start_angle = -np.pi/2
    final_angle =  np.pi/2
    step = 0.01745329238474369

    rango = np.arange(start_angle, final_angle, step)

    scale = 400
    base = np.ones((2*2*scale, 4*scale), dtype=np.uint8) * 255
    points = []

    for n in range(62, 119):
        if muestra[n] == 4.0:
            continue
        x = int(muestra[n]*np.cos(rango[n])*scale)
        y = int(muestra[n]*np.sin(rango[n])*scale)
        base[-y + 2*scale, x] = 0
        points.append((x, y))
    
    h = get_image_angle(points)
    for n in range(len(h)):
        if h[n] < 0:
            h[n] = h[n] + np.pi
    
    angle_ = find_more_common_angle(h)

    basee_point = get_point_with_angle(points, angle_)[0]

    fixed_map = np.ones((2*2*scale, 4*scale), dtype=np.uint8)

    for p in points:
        x, y = p
        ang = get_angle(basee_point, p)
        dist = get_distance(basee_point, p)
        x = int(dist*np.cos(ang - angle_ + np.pi))
        y = int(dist*np.sin(ang - angle_ + np.pi))
        fixed_map[-y + 2*scale, 2*scale + x] = 0
    
    return (fixed_map, angle_)


def get_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return np.arctan2(y2-y1, x2-x1)

def get_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

def get_image_angle(lista):
    angles = []
    for i in range(len(lista)-4):
        angle = get_angle(lista[i], lista[i+4])
        angles.append(angle)
    return angles

def get_point_with_angle(lista, angle):
    for i in range(len(lista)-4):
        if get_angle(lista[i], lista[i+4]) == angle:
            return [lista[i], lista[i+4]]

def find_more_common_angle(lista):
    return max(set(lista), key = lista.count)
