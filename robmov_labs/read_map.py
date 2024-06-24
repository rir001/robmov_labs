#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion
import numpy as np
import cv2

class MapDisplay( Node ):

    def __init__( self ):
        super().__init__( 'map_display' )
        self.mapimg = np.array( [] )
        self.map_resolution = 0
        self.robot_pose = [1.0, 1.0, 0.0]
        self.origin = [0.0, 0.0, 0.0]
        latching_qos = QoSProfile( depth = 1, durability =
        DurabilityPolicy.TRANSIENT_LOCAL )
        self.create_subscription( OccupancyGrid, '/map', self.set_map, qos_profile =
        latching_qos )
        self.create_subscription( Pose, '/real_pose', self.real_pose_cb, 1 )

    def set_map( self, occupancy_grid ):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        self.origin[0] = occupancy_grid.info.origin.position.x
        self.origin[1] = occupancy_grid.info.origin.position.y
        self.map_resolution = occupancy_grid.info.resolution
        print( 'map resolution: (%d,%d)' % (height, width) )
        self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
        self.mapimg = ( self.mapimg * (255/100.0) ).astype( np.uint8 )
        self.mapimg = cv2.cvtColor( self.mapimg, cv2.COLOR_GRAY2RGB )
        self.mapimg = np.flip( self.mapimg, axis = 0 )


    def real_pose_cb( self, pose ):
        if len( self.mapimg ) == 0:
            return None

        x = pose.position.x
        y = pose.position.y
        roll, pitch, yaw = euler_from_quaternion( ( pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w ) )
        self.robot_pose = [x, y, yaw]
        robot_pose_pix_x = int( (self.robot_pose[0] - self.origin[0]) /
        self.map_resolution )
        robot_pose_pix_y = - int( (self.robot_pose[1] - self.origin[1]) /
        self.map_resolution) + self.mapimg.shape[0]
        robot_pose_pix = [robot_pose_pix_x, robot_pose_pix_y, self.robot_pose[2]]
        mapimg_tmp = self.mapimg.copy()
        robot_radio = int( (0.355/2.0) / self.map_resolution )
        cv2.circle( mapimg_tmp, tuple( robot_pose_pix[:2] ), robot_radio, (0, 0, 255), -1 )
        cv2.imshow( '2D Map', mapimg_tmp )
        cv2.waitKey( 1 )


def main():
    rclpy.init()
    map_display = MapDisplay()
    rclpy.spin( map_display )
    map_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
