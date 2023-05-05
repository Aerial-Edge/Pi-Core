#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
from pymavlink_msgs.msg import DronePose
import math


class FollowAlgorithm(Node):
    def __init__(self): 
        super().__init__('follow_algorithm') 

        self.create_subscription(Int32MultiArray, 'object_pos_and_distance', self.position_and_distance_callback, 10)
        
        self.create_subscription(DronePose, 'drone_pose', self.qualisys_callback, 10)
        self.distance = 0.0
        
        
    def qualisys_callback(self, msg: DronePose):

        

        # Constants
        object_x = 0.0  # meters
        object_y = 0.0  # meters
        camera_angle = 45  # degrees
        
        # Variables
        yaw = msg.yaw.data # radians -pi to pi
        true_x = msg.pos.x # x position in meters from qualisys
        true_y = msg.pos.y # y position in meters from qualisys
        distance_to_object = self.distance# meters




        # Calculate the drone's position
        drone_x, drone_y = self.find_drone_position(yaw , distance_to_object, object_x, object_y, camera_angle)
        
        self.get_logger().info(f"Estimated position {drone_x} {drone_y} ")
        self.get_logger().info(f"True position {true_x} {true_y} ")



    def find_drone_position(self, yaw, distance_to_object, object_x, object_y, camera_angle):
    
        
        # Calculate drone position
        horizontal_distance = distance_to_object * math.cos(math.radians(camera_angle))
        drone_x = object_x + horizontal_distance * math.cos(yaw)
        drone_y = object_y + horizontal_distance * math.sin(yaw)

        return drone_x, drone_y


    def position_and_distance_callback(self, msg: Int32MultiArray):

        self.distance = msg.data[2]
        self.get_logger().info(f"Distance to object {self.distance} ")
       

    


def main(args=None):
    rclpy.init(args=args)
    follow_algorithm = FollowAlgorithm()
    rclpy.spin(follow_algorithm)
    follow_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
