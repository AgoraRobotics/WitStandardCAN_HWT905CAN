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
    set_pub = node.create_publisher(String, 'wit/cali', 10)
    print("please input your cmd:")
    showhelp()
    while rclpy.ok():
        try:
            string = input("input cmd:")
            if 'rate' in string:
                set_pub.publish(String(data=string))
                print("change " + string)
            elif 'rsw' in string:
                set_pub.publish(String(data='rsw'))
            elif 'baud' in string:
                set_pub.publish(String(data=string))
                print("change " + string)
            elif 'b' in string:
                set_pub.publish(String(data="begin"))
                print("begin recording")
            elif 's' in string:
                set_pub.publish(String(data="stop"))
                print("stop recording")
            elif 'v' in string:
                set_pub.publish(String(data="version"))
                print("show sensor version")
            elif 'h' in string:
                showhelp()
            elif 'e' in string:
                print("exti sys\n")
                exit(0)
            elif '0' in string:
                set_pub.publish(String(data="exti"))
                print("exti cali mode")
            elif '9' in string:
                set_pub.publish(String(data="mag"))
                print("enter mag cali mode")
            else :
                print("{} cmd no support".format(string))
        except Exception as e:
            print(e)
        rclpy.spin_once(node)
    rclpy.shutdown()
