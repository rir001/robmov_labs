#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty


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

        self.actuation_pub = self.create_publisher( Float64, f'velocity{"_"+name if name else ""}', 1 )
        self.dist_set_point_sub = self.create_subscription( Float64, f'setpoint{"_"+name if name else ""}', self.setpoint_cb, 1 )
        self.dist_state_sub = self.create_subscription( Float64, f'state{"_"+name if name else ""}', self.state_cb, 1 )

    def setpoint_cb( self, msg ):
        self.get_logger().info(f'[PID{"_"+self.name if self.name else ""}] new setpoint received: %.2f' % (msg.data) )
        self.reset()
        self.setpoint = msg.data

    def state_cb( self, msg ):
        if self.setpoint == None:
            return
        self.state = msg.data
        error = self.setpoint - self.state

        p_actuation = self.kp*error
        i_actuation = self.ki*self.cumulated_error
        d_actuation = self.kd*(self.past_error + error)/2

        self.past_error = error
        self.cumulated_error += error

        actuation = p_actuation + i_actuation + d_actuation

        msg = Float64()
        msg.data = actuation
        self.actuation_pub.publish( msg )

    def reset( self ):
        self.setpoint = None
        self.state = None
        self.past_error = 0
        self.cumulated_error = 0

def run_pid_desp():
    rclpy.init()
    pid_desp_ctrl = PIDController( 0.5, name="desp")
    rclpy.spin( pid_desp_ctrl )

def run_pid_angle():
    rclpy.init()
    p_angle_ctrl = PIDController( 0.5, name="angle")
    rclpy.spin( p_angle_ctrl  )

def main():
    rclpy.init()
    p_ctrl = PIDController( 0.5 )
    rclpy.spin( p_ctrl )

if __name__ == '__main__':
    main()
