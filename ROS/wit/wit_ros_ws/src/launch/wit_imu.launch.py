from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	# Imu model, default normal -> converted to default CAN
	# 	If the device type is modbus protocol, fill in modbus
	# 	If the device type is wit standard protocol, fill in normal
	# 	If the device type is MODBUS high-precision protocol, fill in hmodbus
	# 	If the device type is CAN protocol, fill in can
	# 	If the device type is CAN high-precision protocol, fill in hcan
	# 	The device number/dev/ttyUSB0 (the default script uses/dev/ttyUSB0) is the number recognized by your computer
	# 	The baud rate is set according to actual usage. The default baud rate for JY6x series modules is 115200, CAN modules are 230400, and other modules are 9600
	# 	If the user modifies the baud rate through the upper computer, it needs to be correspondingly modified to the modified baud rate 

    return LaunchDescription([
        Node(
            package='wit_ros_imu',
            executable='wit_can_ros.py', # change this to wit_normal_ros.py, wit_modbus_ros.py, wit_hmodbus_ros.py, wit_hcan_ros.py
            name='imu',
            output='screen',
            parameters=[
                {"port": "/dev/ttyUSB0"},
                {"baud": 115200}
            ]
        )
    ])