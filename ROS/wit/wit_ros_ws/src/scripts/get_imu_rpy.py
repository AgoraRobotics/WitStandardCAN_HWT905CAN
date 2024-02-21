#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf_transformations import *
from sensor_msgs.msg import Imu


def callback(self, data):
    # This function is in tf and can convert quaternions into Euler angles
    (r, p, y) = euler_from_quaternion((data.orientation.x,
                                       data.orientation.y, data.orientation.z, data.orientation.w))

    # Since it is in radian system, it seems more convenient to change it to angle system below.
    self.get_logger().info('Roll: "%f", Pitch: "%f", Yaw: "%f"' %
                           (r*180/3.1415926, p*180/3.1415926, y*180/3.1415926))


def get_imu():
    rclpy.init()
    node = rclpy.create_node('get_imu')
    subscription = node.create_subscription(Imu, '/wit/imu', callback, 10)
    subscription  # prevent unused variable warning
    rclpy.spin(node)

if __name__ == '__main__':
    get_imu()
    rclpy.shutdown()
    pass
