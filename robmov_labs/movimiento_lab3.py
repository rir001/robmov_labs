#!/usr/bin/env python3
import numpy as np
from numpy import pi
np.float = float

import rclpy
# from time import sleep
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan


OMEGA = 1.0          # rad/s
T_AJUSTE = 1.1085    # factor de ajuste para el tiempo de duracion de los movimientos
VEL = 0.2            # m/s
WINDOW = 1
M1 = 0.1    # Margin 1, 5.7°
M2 = 0.2    # Margin 2, 11.5°
DM = 0.02    # Distance Margin, 0.05m
CENTER = 62 + int((118 - 62)/2) + 1
FM = 0.4   # Forward Movement, 0.25m


############################
### Funciones Auxiliares ###
############################


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

def gira(angulo: float):
    '''
    En base a un angulo (en radianes), se retorna el comando de giro necesario para que el robot gire ese angulo.
    Retorna una lista con la velocidad lineal, velocidad angular y tiempo de duracion del movimiento [v, w, t].
    '''
    tiempo = abs(float(angulo/OMEGA)*T_AJUSTE)
    return [0.0, OMEGA, tiempo]
    
def avanza(distancia: float):
    '''
    En base a una distancia, se retorna el comando de avance necesario para que el robot avance esa distancia.
    Retorna una lista con la velocidad lineal, velocidad angular y tiempo de duracion del movimiento [v, w, t].
    '''
    tiempo = float(distancia/VEL)
    return [VEL, 0.0, tiempo]


class Lab3_Nav( Node ):

    def __init__( self ):
        super().__init__( 'movimiento_lab3' )

        self.cmd_vel_mux_pub            = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )
        self.lidar_sub                  = self.create_subscription( LaserScan, '/scan', self.lidar_cb, 1 )
        self.particles_filter_sub       = self.create_subscription( PoseArray, '/particles', self.pf_cb, 1 )
        self.particles_filter_pub       = self.create_publisher( PoseArray, '/moved_particles', 1 )

        self.speed = Twist()

        self.angles = []
        self.acciones = []
        self.rango = np.arange(-np.pi/2, np.pi/2, step=0.01745329238474369)
        self.distancia = 0
        self.pause_robot = True
        self.timer_period = 0.2 # seconds

    
    def lidar_cb(self, image):
        # self.get_logger().info(f'image.ranges: {list(image.ranges)}')
        muestra = list(image.ranges)
        points = [(0, 0)]
        distancias = []
        accion = False
        
        for n in range(CENTER-7, CENTER+7):
            if muestra[n] == 4.0: continue
            x = muestra[n]*np.cos(self.rango[n])
            y = muestra[n]*np.sin(self.rango[n])
            distancias.append(x)

            points.append((x, y))

        # Distancia
        if len(distancias) == 14:
            if len([d for d in distancias if abs(d - np.mean(distancias)) < DM]) >= 11:
                accion = 1
                self.distancia = sorted(distancias)[2] # es el 3 menor valor de la lista
            else:
                accion = 0

        if len(self.acciones) >= 10:
            self.acciones.pop(0)
        if accion == 0 or accion == 1:
            self.acciones.append(accion)

        # self.get_logger().info(f'distancias: {distancias}, distancia: {self.distancia}')
        # self.get_logger().info(f'acciones: {self.acciones}, accion: {accion}')

        # Ángulo
        h = get_aprox_image_angles(points)
        if len(h) > 0:
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
        # return 
        if len(self.angles) > 0:
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
            
            self.get_logger().info(f'Selected angle: {selected_angle}, acciones: {self.acciones}')
            if sum(self.acciones) >= 5:
                # Avanzamos min(X y la distancia a la pared - 0.15)
                movement = min(FM, self.distancia - 0.1)

                # Nos aseguramos de avanzar y luego actualizamos las particulas
                if movement < FM:   # En caso de que la distancia sea menor a FM, giramos 90° a la derecha
                    self.speed.linear.x, self.speed.angular.z, execution_time = gira(-np.pi/2)
                    updated_particles = self.actualizar_particulas(msg.poses, -np.pi/2, 0)
                else:
                    self.speed.linear.x, self.speed.angular.z, execution_time = avanza(movement)
                    updated_particles = self.actualizar_particulas(msg.poses, movement, 1)
                
            else:
                # Giramos hasta estar perpendicular a la pared
                self.speed.linear.x, self.speed.angular.z, execution_time = gira(selected_angle)
                updated_particles = self.actualizar_particulas(msg.poses, selected_angle, 0)
            
            # Publicar el movimiento a realizar
            self.pause_robot = False
            self.aplicar_velocidad(execution_time)

            # IMPORTANTE ---> Lo cumple el método aplicar_velocidad
            # De alguna forma verificar que ya se haya movido el robot antes de publicar las nuevas particulas
            # (para asegurarse de que se lea la nueva pose en el momento adecuado)
            # IMPORTANTE
            # sleep(2)

            # Publicar las nuevas particulas (aplicando el movimiento del robot a las particuals anteriores)
            new_particles = PoseArray()
            new_particles.header = msg.header
            new_particles.poses = updated_particles
            self.particles_filter_pub.publish(new_particles)
    
    def aplicar_velocidad(self, t_ejecucion):
        t_inicial = self.get_clock().now().nanoseconds
        t_actual = self.get_clock().now().nanoseconds
        contador = 0

        self.get_logger().info(f'Executing movement for {t_ejecucion} seconds')
        delta = 0
        while t_actual - t_inicial - delta < t_ejecucion*10**9:
        
            pause = True
            while self.pause_robot:
                if pause:
                    self.get_logger().info("Robot paused")
                    p_start = self.get_clock().now().nanoseconds
                pause = False
            if not pause:
                delta += self.get_clock().now().nanoseconds - p_start

            if (t_actual - t_inicial - delta) > self.timer_period*contador*10**9:
                if pause:
                    contador += 1
                self.cmd_vel_mux_pub.publish(self.speed)
                self.get_logger().info(f'Moving: v={self.speed.linear.x}, w={self.speed.angular.z},     {contador}')
                
            t_actual = self.get_clock().now().nanoseconds
        
        self.get_logger().info(f'Finished movement')
        self.speed.linear.x, self.speed.angular.z = 0.0, 0.0
        self.cmd_vel_mux_pub.publish(self.speed)
        self.pause_robot = True

            
    def actualizar_particulas(self, particles, value, movement):
        '''
        Actualiza las particulas según el movimiento que haya sido realizado (avance o giro).

        'value' es la distancia avanzada o el ángulo girado.
        'movement' es el tipo de movimiento realizado (0: giro, 1: avance).
        '''
        new_particles = []
        for p in particles:
            x, y = p.position.x, p.position.y
            q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            _, _, a = euler_from_quaternion(q)
            if movement:
                x += value*np.cos(a)
                y += value*np.sin(a)
            else:
                a += value
            new_q = quaternion_from_euler(0, 0, a)
            new_particles.append(Pose(
                position = Point(x=x, y=y, z=float(0)),
                orientation = Quaternion(x=new_q[0], y=new_q[1], z=new_q[2], w=new_q[3])
            ))
        
        return new_particles
        


def main():
    rclpy.init()
    node = Lab3_Nav()
    rclpy.spin( node )


if __name__ == '__main__':
    main()


