#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class VirtualRobot( Node ):

  def __init__( self ):
    super().__init__( 'virtual_robot' )
    self.x_pos = 0.0
    self.speed = 0.0

    self.dist_set_point = self.create_publisher( Float64, 'setpoint', 1 )
    while self.dist_set_point.get_subscription_count() == 0 and rclpy.ok():
      time.sleep( 0.2 )

    self.dist_state = self.create_publisher( Float64, 'state', 1 )
    while self.dist_state.get_subscription_count() == 0 and rclpy.ok():
      time.sleep( 0.2 )

    self.actuation = self.create_subscription( Float64, 'control_effort', self.dist_actuation, 1 )

    self.period = 0.1
    self.timer = self.create_timer(self.period, self.virtual_odom)

  def virtual_odom( self ):
    self.x_pos += self.speed*(self.period)
    msg = Float64()
    msg.data = self.x_pos
    self.dist_state.publish( msg )
    self.get_logger().info( 'speed received: %.4f - current position: %.4f' % ( self.speed, self.x_pos ) )

  def dist_actuation( self, data ):
    self.speed = float( data.data )

  def move_forward( self, set_point ):
    msg = Float64()
    msg.data = set_point
    self.dist_set_point.publish( msg )


def main():
  rclpy.init()
  vrobot = VirtualRobot()
  vrobot.move_forward( 3.0 )
  rclpy.spin( vrobot )

if __name__ == '__main__':
  main()



