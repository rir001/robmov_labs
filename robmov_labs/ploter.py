#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
np.float = float

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class Ploter( Node ):

    def __init__(self, name = None):

        super().__init__( f'Grafico{"_"+name if name else ""}'  )

        self.actuations = []
        self.setpoints = []
        self.states = []
        self.x = []

        self.actuation_sub =        self.create_subscription( Float64, f'velocity{"_"+name if name else ""}',self.add_actuation, 1 )
        self.dist_set_point_sub =   self.create_subscription( Float64, f'setpoint{"_"+name if name else ""}', self.add_setpoint, 1 )
        self.dist_state_sub =       self.create_subscription( Float64, f'state{"_"+name if name else ""}', self.add_state_cb, 1 )



        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], lw=2, color="red", label="Actuador")
        self.line2, = self.ax.plot([], [], lw=2, color="blue", label="Referencia")
        self.line3, = self.ax.plot([], [], lw=2, color="green", label="Real")
        self.ax.clear()

        self.anim = FuncAnimation(self.fig, self.actualizar, blit=False, interval=1000)
        plt.show()
    
    def add_actuation(self, value):
        self.actuations.append(value)

    def add_setpoint(self, value):
        self.setpoints.append(value)
    
    def add_state_cb(self, value):
        self.states.append(value)

    def actualizar(self):
        self.x = np.arange(len(self.actuations))
        self.line1.set_data(self.x, self.actuations)
        self.line2.set_data(self.x, self.setpoints)
        self.line3.set_data(self.x, self.states)
        return self.line1, self.line2, self.line3

    def init(self):
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        return self.line1, self.line2, self.line3
    

def plot_desp():
    rclpy.init()
    ploter_desp = Ploter( name="desp")
    rclpy.spin( ploter_desp )

def plot_angle():
    rclpy.init()
    ploter_ang = Ploter( name="angle")
    rclpy.spin( ploter_ang  )


def main():
    rclpy.init()
    ploter = Ploter()
    rclpy.spin( ploter )

if __name__ == '__main__':
    main()

    




