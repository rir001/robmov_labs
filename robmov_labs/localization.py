#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_msgs.msg import Bool

import cv2
import matplotlib.pyplot as plt

import os


VEL = 0.4
PLOT = 1
PARTICLES = 100_000

SCALE = 100
RANGO_ANGULAR = np.arange(-np.pi/2, np.pi/2, 0.01745329238474369)

REAL_MAP = cv2.imread("./src/robmov_labs/maps/mapa.pgm", cv2.IMREAD_GRAYSCALE)

maskm_map = REAL_MAP.copy()

maskm_map[maskm_map == 0] = 255
maskm_map[maskm_map == 205] = 0
maskm_map = cv2.erode(maskm_map, np.ones([19]*2, np.uint8), iterations=1)
maskm_map = (np.logical_not(REAL_MAP).astype(np.uint8) * maskm_map / 255).astype(np.uint8)
DARK_MAP = cv2.dilate(maskm_map, np.ones([3]*2, np.uint8), iterations=1)


def blur_image(image, kernel_size):

    n = kernel_size
    sigma = n/3
    array = np.arange(n) + 1
    kernel = np.exp(-((array ** 2 ) / (2.0 * sigma ** 2)))
    A = np.ones(image.shape)

    final = image.copy()*A
    dilatada = image.copy()


    for i in range(n):

        dilatada_a = dilatada.copy()
        dilatada = cv2.dilate(dilatada, np.ones((3, 3), np.uint8))
        capa = (dilatada-dilatada_a) * kernel[i]
        if i == 0:
            final += dilatada * kernel[i]
        final += capa

    return  final

mask = REAL_MAP.copy()

mask[mask == 0] = 255
mask[mask == 205] = 0

GAUSSIAN_MAP = blur_image(DARK_MAP, 20) * cv2.erode(mask, np.ones([3]*2, np.uint8), iterations=7)
GAUSSIAN_MAP /= GAUSSIAN_MAP.max()

def get_particles(N:int, points=[], r:int=10, angle_tolerance:int=0.1) -> np.ndarray:
    angles = np.linspace(-np.pi, np.pi, int(360/angle_tolerance))

    if list(points) == []:
        spaces = np.column_stack(np.where(REAL_MAP == 255))
    else:
        options = np.zeros(REAL_MAP.shape, dtype=np.uint8)
        for point in points: options[int(point[0])-r:int(point[0])+r+1, int(point[1])-r:int(point[1])+r+1] = 1
        spaces = np.column_stack(np.where((options * REAL_MAP) == 255))

    return np.column_stack([
            spaces[np.random.randint(spaces.shape[0] , size=N), :],
            np.random.choice(angles, size=N),
        ])

def get_more_correct_particles(particles:list, muestra:list, tolerance:int=1, umbral:int=3):
    sobrevivientes = []

    step = np.arange(62, 119, 1)

    for y, x, a in particles:
        y_a = (y - SCALE*muestra[step]*np.sin(RANGO_ANGULAR[step] + a)).astype(int)
        x_a = (x + SCALE*muestra[step]*np.cos(RANGO_ANGULAR[step] + a)).astype(int)

        if ((15 < y_a) & (y_a < 255) & (15 < x_a-tolerance) & (x_a < 255)).all():
            if np.product(GAUSSIAN_MAP[y_a, x_a]) > umbral:
                sobrevivientes.append([y, x, a])

    return np.array(sobrevivientes)


class Localization( Node ):

    def __init__( self ):
        super().__init__( 'localization' )

        self.ploted = False

        self.lidar_sub                      = self.create_subscription( LaserScan, '/scan', self.process_lidar,  1 )
        self.cmd_vel_mux_pub                = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )
        self.particle_pub                   = self.create_publisher( PoseArray, '/particles', 1 )
        self.mvmnt_rdy_sub                  = self.create_subscription( PoseArray, '/moved_particles', self.move, 1 )
        self.particles_filter_flag_sub      = self.create_subscription( Bool, '/particles_filter_flag', self.particles_filter_flag, 1 )

        self.bg = CvBridge()
        self.first = 1
        self.wait = 0


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
        # self.get_logger().info( f"{ image.ranges }" )

    def publish_particles(self, particles):
        poses = []
        for p in particles:
            q = quaternion_from_euler(0, 0, p[2])
            poses.append(Pose(
                position = Point(x=float(p[1]/SCALE), y=float((270 - p[0])/SCALE), z=float(0)),
                orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            ))
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.poses = poses
        self.particle_pub.publish(msg)

    def process_lidar(self, image):
        if PLOT: self.show_lidar(image)

        if self.first:
            particles = get_particles(PARTICLES)
            live_particles = get_more_correct_particles(particles, np.array(image.ranges), umbral=.000001)
            particles = np.vstack([
                get_particles(int(PARTICLES*0.95), points=live_particles, angle_tolerance=0.001),
                get_particles(int(PARTICLES*0.05)),
            ])
            live_particles = get_more_correct_particles(particles, np.array(image.ranges), umbral=.00001)
            # self.get_logger().info( f"{ live_particles }" )

            self.publish_particles(live_particles)
            self.first = False
        else:
            if self.wait:
                # self.get_logger().info(f"particulas: { self.new_particles }")
                particles = np.vstack([
                    get_particles(int(PARTICLES*0.99), points=self.new_particles),
                    get_particles(int(PARTICLES*0.01)),
                ])
                live_particles = get_more_correct_particles(particles, np.array(image.ranges), umbral=.000001)
                # self.get_logger().info( f"{ live_particles }" )
                self.publish_particles(live_particles)
                if 0 < len(live_particles) < 10:
                    # self.get_logger().info(f" {live_particles} ")
                    # self.get_logger().info(f" {live_particles.shape} ")
                    # if len(live_particles) == 1:
                    #     self.get_logger().info(f"Estoy en Y:{ (live_particles[0][0])/SCALE }, X:{ (live_particles[0][1])/SCALE }")
                    #     self.get_logger().info(f"Con un angulo: { (live_particles[0][:, 2]) }")
                    # else:
                    self.get_logger().info(f"Estoy en Y:{ (np.mean(live_particles[:, 0])/SCALE) }, X:{ (np.mean(live_particles[:, 1])/SCALE) }")
                    self.get_logger().info(f"Con un angulo: { (np.mean(live_particles[:, 2])) }")
                self.wait = False


            # self.wait = True
    def particles_filter_flag(self, msg):
        self.wait = msg.data

    def move(self, particles):
        # self.get_logger().info( "Move" )
        # self.get_logger().info( f"{ particles }" )

        new_particles = []
        for p in particles.poses:
            x, y = p.position.x, p.position.y
            q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            _, _, a = euler_from_quaternion(q)
            new_particles.append([270 - y*SCALE, x*SCALE, a])
        self.new_particles = np.array(new_particles)




def main():
    rclpy.init()
    node = Localization()
    rclpy.spin( node )


if __name__ == '__main__':
    main()
