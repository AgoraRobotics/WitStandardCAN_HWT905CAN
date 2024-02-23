#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

def callback(data):
    rclpy.logging.get_logger('Set').info(data.data)

def showhelp():
	print("----------------------------")
	print("0:exti cali mode")
	print("9:enter mag cali mode")
	print("h:show cmd help")
	print("e:exti sys")
	print("v:show version")
	print("b:begin recording")
	print("s:stop recording")
	print("rate:set 0.2~200Hz output")
	print("baud:set 4800~230400 baud")
	print("rsw:set output data <time,acc,gyro,angle,mag>")
	print("----------------------------")

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('Set')
    set_pub = node.create_publisher(String, '/wit/cali', 10)
    showhelp()
    
    node.get_logger().info("please input your cmd:")
    
    while rclpy.ok():
        node.get_logger().info("please input your cmd:")
        string = input()
        if 'rate' in string:
            set_pub.publish(String(data=string))
            node.get_logger().info("change " + string)
        elif 'rsw' in string:
            set_pub.publish(String(data='rsw'))
            node.get_logger().info("change " + string)
        elif 'baud' in string:
            set_pub.publish(String(data=string))
            node.get_logger().info("change " + string)
        elif 'b' in string:
            set_pub.publish(String(data="begin"))
            node.get_logger().info("begin recording")
        elif 's' in string:
            set_pub.publish(String(data="stop"))
            node.get_logger().info("stop recording")
        elif 'v' in string:
            set_pub.publish(String(data="version"))
            node.get_logger().info("show sensor version")
        elif 'h' in string:
            showhelp()
        elif 'e' in string:
            node.get_logger().info("exti sys\n")
            exit(0)
        elif '0' in string:
            set_pub.publish(String(data="exti"))
            node.get_logger().info("exti cali mode")
        elif '9' in string:
            set_pub.publish(String(data="mag"))
            node.get_logger().info("enter mag cali mode")
        else :
            node.get_logger().info("{} cmd no support".format(string))
        rclpy.spin(node)
    rclpy.shutdown()
