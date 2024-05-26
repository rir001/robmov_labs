#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


MODE = "PI"


class PIDController( Node ):

    def __init__( self, kp, ki = 0, kd = 0, name=None):
        super().__init__( f'pid{"_"+name if name else ""}'  )
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.name = name
        self.setpoint = None
        self.state = None
        self.proportional_action = 0

        self.actuation_pub =        self.create_publisher( Float64, f'velocity{"_"+name if name else ""}', 1 )
        self.dist_set_point_sub =   self.create_subscription( Float64, f'setpoint{"_"+name if name else ""}', self.setpoint_cb, 1 )
        self.dist_state_sub =       self.create_subscription( Float64, f'state{"_"+name if name else ""}', self.state_cb, 1 )

    def setpoint_cb( self, msg ):
        self.get_logger().info(f'[PID{"_"+self.name if self.name else ""}] new setpoint received: %.2f' % (msg.data) )
        self.reset()
        self.setpoint = msg.data

    def state_cb( self, msg ):
        if self.setpoint == None: return

        self.state = msg.data
        error = self.setpoint - self.state

        p_actuation = self.kp*error
        i_actuation = self.ki*sum(self.cumulated_error)
        d_actuation = self.kd*(self.past_error + error)/2 #implementar con tiempo

        self.past_error = error
        self.cumulated_error.append(error)
        if len(self.cumulated_error) > 20: self.cumulated_error.pop(0)

        actuation = p_actuation + i_actuation + d_actuation

        msg = Float64()
        msg.data = actuation
        self.actuation_pub.publish( msg )

    def reset( self ):
        self.get_logger().info(f'{self.name} error: {self.setpoint - self.state if self.state and self.setpoint else "None"}' )
        self.setpoint = None
        self.state = None
        self.past_error = 0
        self.cumulated_error = []

def run_pid_desp():
    rclpy.init()
    if MODE == "P":
        pid_desp_ctrl = PIDController( kp=16, name="desp")
    if MODE == "PI":
        pid_desp_ctrl = PIDController( kp=1.6, ki=0.8, name="desp")
    rclpy.spin( pid_desp_ctrl )

def run_pid_angle():
    rclpy.init()
    if MODE == "P":
        p_angle_ctrl = PIDController( kp=5.5, name="angle")
    if MODE == "PI":
        p_angle_ctrl = PIDController( kp=2, ki=0.05 , name="angle")
    rclpy.spin( p_angle_ctrl  )

def main():
    rclpy.init()
    p_ctrl = PIDController( 0.5 )
    rclpy.spin( p_ctrl )

if __name__ == '__main__':
    main()
