#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
from pymavlink_msgs.msg import Attitude, DronePose


class FollowAlgorithm(Node):
    def __init__(self): 
        super().__init__('follow_algorithm') 

        self.create_subscription(Int32MultiArray, 'object_pos_and_distance', self.position_and_distance_callback, 10)
        
        self.create_subscription(DronePose, 'drone_pose', self.qualisys_callback, 10)
        
        
    def qualisys_callback(self, msg: DronePose):

        self.get_logger().info(f" {msg.pos}")



    def position_and_distance_callback(self, msg: Int32MultiArray):

        self.get_logger().info(f" {msg.data} ")

    


def main(args=None):
    rclpy.init(args=args)
    follow_algorithm = FollowAlgorithm()
    rclpy.spin(follow_algorithm)
    follow_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
