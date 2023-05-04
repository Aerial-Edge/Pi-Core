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
import math


class FollowAlgorithm(Node):
    def __init__(self): 
        super().__init__('follow_algorithm') 

        self.create_subscription(Int32MultiArray, 'object_pos_and_distance', self.position_and_distance_callback, 10)
        
        self.create_subscription(DronePose, 'drone_pose', self.qualisys_callback, 10)
        
        
    def qualisys_callback(self, msg: DronePose):

        yaw = msg.yaw.data

        #self.get_logger().info(f" {msg.yaw}")

         # Constants
        room_width = 10  # meters
        room_length = 10  # meters

        camera_angle = 45  # degrees

        def radians_to_degrees(yaw_radians):
            # Convert Float32 to Python float
            yaw_radians_float = float(yaw_radians)

            # Convert radians to degrees
            yaw_degrees = math.degrees(yaw_radians_float)

            # Normalize the angle to the range [0, 360)
            yaw_degrees_normalized = (yaw_degrees + 360) % 360

            return yaw_degrees_normalized

        def calculate_object_position(yaw, distance):
            # Convert yaw to radians
            yaw_rad = math.radians(yaw)
            # Calculate the horizontal distance from the drone to the object
            horizontal_distance = distance * math.cos(math.radians(camera_angle))

            # Calculate object position
            object_x = horizontal_distance * math.cos(yaw_rad)
            object_y = horizontal_distance * math.sin(yaw_rad)

            return object_x, object_y

        def find_drone_position(yaw, distance_to_object):
            # Calculate object position
            object_x, object_y = calculate_object_position(yaw, distance_to_object)

            # Calculate drone position
            drone_x = object_x + room_width / 2
            drone_y = object_y + room_length / 2

            return drone_x, drone_y

        yaw_degree = radians_to_degrees(yaw)
        distance_to_object = 5
        # Calculate the drone's position
        drone_x, drone_y = find_drone_position(yaw_degree, distance_to_object)
        
        self.get_logger().info(f" {drone_x} {drone_y} {yaw_degree} ")




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
