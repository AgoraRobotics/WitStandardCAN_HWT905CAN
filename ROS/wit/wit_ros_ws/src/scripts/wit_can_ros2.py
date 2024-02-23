#!/usr/bin/env python3

import struct
import time
import math
import threading
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

readreg = 0
key = 0
flag = 0
iapflag = 0
recordflag = 0
buff = {}
calibuff = []
recordbuff = []
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
mag_offset = [0, 0, 0]
mag_range = [0, 0, 0]
baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu')
        self.subscription = self.create_subscription(
            String,
            '/wit/cali',
            self.callback,
            10)
        self.find_ttyUSB()
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 230400)
        
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baud").value
        self.get_logger().info("IMU Type: HCAN Port:%s baud:%d" % (self.port, self.baudrate))
        
        self.imu_pub = self.create_publisher(Imu, 'wit/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'wit/mag', 10)
  


    def find_ttyUSB(self):
        self.get_logger().info('The default serial port of the imu is /dev/ttyUSB0, if multiple serial port devices are identified, modify the serial port corresponding to the imu in the launch file')
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
        self.get_logger().info('There are {} {} serial port devices connected to the current PC: {}'.format(len(posts), 'USB', posts))

    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhh", bytearray(raw_data)))

    def handle_serial_data(self, raw_data):
        global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, readreg, calibuff, flag, mag_offset, mag_range
        global angle_tip
        
        angle_flag = False
        buff[key] = raw_data
        key += 1
        
        self.get_logger().info("key: %d" % key)
        self.get_logger().info("Handle Serial Data")
        
        if buff[0] != 0x55:
            key = 0
            self.get_logger().info("buff[0] != 0x55")
            return
        if key < 8:
            self.get_logger().info("key < 8")
            return
        else:
            data_buff = list(buff.values())
            if buff[1] == 0x51:
                acceleration = [self.hex_to_short(data_buff[2:8])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                self.get_logger().info(acceleration)
            elif buff[1] == 0x52:
                angularVelocity = [self.hex_to_short(data_buff[2:8])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
            elif buff[1] == 0x53:
                temp = self.hex_to_short(data_buff[2:8])
                angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
            elif buff[1] == 0x54:
                magnetometer = self.hex_to_short(data_buff[2:8])
                if flag:
                    calibuff.append(magnetometer[0:2])
            elif buff[1] == 0x5f:
                readval = self.hex_to_short(data_buff[2:8])
                if readreg == 0x0b:
                    mag_offset = readval
                else:
                    mag_range = readval
                self.get_logger().info(readval)
            else:
                buff = {}
                key = 0
            buff = {}
            key = 0
            if angle_flag:
                stamp = self.get_clock().now().to_msg()
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "base_link"
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = "base_link"
                angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
                qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
                imu_msg.orientation.x = qua[0]
                imu_msg.orientation.y = qua[1]
                imu_msg.orientation.z = qua[2]
                imu_msg.orientation.w = qua[3]
                imu_msg.angular_velocity.x = angularVelocity[0]
                imu_msg.angular_velocity.y = angularVelocity[1]
                imu_msg.angular_velocity.z = angularVelocity[2]
                imu_msg.linear_acceleration.x = acceleration[0]
                imu_msg.linear_acceleration.y = acceleration[1]
                imu_msg.linear_acceleration.z = acceleration[2]
                mag_msg.magnetic_field.x = magnetometer[0]
                mag_msg.magnetic_field.y = magnetometer[1]
                mag_msg.magnetic_field.z = magnetometer[2]
                self.imu_pub.publish(imu_msg)
                self.get_logger().info("Published IMU")
                self.mag_pub.publish(mag_msg)
                angle_tip = 0

    def callback(self, data):
        global readreg, flag, calibuff, wt_imu, iapflag, mag_offset, mag_range, version, recordflag, baudlist
        unlock_imu_cmd = b'\xff\xaa\x69\x88\xb5'
        reset_magx_offset_cmd = b'\xff\xaa\x0b\x00\x00'
        reset_magy_offset_cmd = b'\xff\xaa\x0c\x00\x00'
        reset_magz_offset_cmd = b'\xff\xaa\x0d\x00\x00'
        enter_mag_cali_cmd = b'\xff\xaa\x01\x09\x00'
        exti_cali_cmd = b'\xff\xaa\x01\x00\x00'
        save_param_cmd = b'\xff\xaa\x00\x00\x00'
        read_mag_offset_cmd = b'\xff\xaa\x27\x0b\x00'
        read_mag_range_cmd = b'\xff\xaa\x27\x1c\x00'
        reboot_cmd = b'\xff\xaa\x00\xff\x00'
        reset_mag_param_cmd = b'\xff\xaa\x01\x07\x00'
        set_rsw_demo_cmd = b'\xff\xaa\x02\x1f\x00'
        self.get_logger().info('callback')
        self.get_logger().info(data.data)
        if "mag" in data.data:
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magx_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magy_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magz_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_mag_param_cmd)
            time.sleep(0.1)
            self.wt_imu.write(enter_mag_cali_cmd)
            time.sleep(0.1)
            flag = 1
            calibuff = []
            mag_offset = [0, 0, 0]
            mag_range = [500, 500, 500]
        elif "exti" in data.data:
            flag = 0
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(exti_cali_cmd)
            time.sleep(0.1)
            self.wt_imu.write(save_param_cmd)
            time.sleep(1)
            readreg = 0x0b
            self.wt_imu.write(read_mag_offset_cmd)
            time.sleep(1)
            readreg = 0x1c
            self.wt_imu.write(read_mag_range_cmd)
            time.sleep(1)
            datalen = len(calibuff)
            self.get_logger().info('cali data {}'.format(datalen))
            r = []
            if datalen > 0:
                for i in range(datalen):
                    tempx = ((calibuff[i][0] - mag_offset[0]) * 2 / float(mag_range[0]))
                    tempy = ((calibuff[i][1] - mag_offset[1]) * 2 / float(mag_range[1]))
                    temp = tempx * tempx + tempy * tempy - 1
                    r.append(abs(temp))
                sumval = sum(r)
                r_n = float(sumval) / datalen
                if r_n < 0.05:
                    self.get_logger().info('magnetic field calibration results are very good')
                elif r_n < 0.1:
                    self.get_logger().info('magnetic field calibration results are good')
                else:
                    self.get_logger().info('magnetic field calibration results is bad, please try again')
        elif "version" in data.data:
            self.get_logger().info('sensor version is {}'.format(version))
        elif "begin" in data.data:
            record_thread = threading.Thread(target=self.record_thread)
            record_thread.start()
        elif "stop" in data.data:
            recordflag = 0
        elif "rate" in data.data:
            ratelist = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200]
            try:
                val = data.data[4:]
                rate = float(val)
                for i in range(len(ratelist)):
                    if rate == ratelist[i]:
                        self.get_logger().info('chage {} rate'.format(rate))
                        val = i + 1
                        cmd = bytearray(5)
                        cmd[0] = 0xff
                        cmd[1] = 0xaa
                        cmd[2] = 0x03
                        cmd[3] = val
                        cmd[4] = 0x00
                        self.wt_imu.write(unlock_imu_cmd)
                        time.sleep(0.1)
                        self.wt_imu.write(cmd)
            except Exception as e:
                self.get_logger().info(e)
        elif "baud" in data.data:
            try:
                val = data.data[4:]
                baud = float(val)
                for i in range(len(baudlist)):
                    if baud == baudlist[i]:
                        val = i + 1
                        cmd = bytearray(5)
                        cmd[0] = 0xff
                        cmd[1] = 0xaa
                        cmd[2] = 0x04
                        cmd[3] = val
                        cmd[4] = 0x00
                        self.wt_imu.write(unlock_imu_cmd)
                        time.sleep(0.1)
                        self.wt_imu.write(cmd)
                        time.sleep(0.1)
                        self.wt_imu.baudrate = baud
            except Exception as e:
                self.get_logger().info(e)
        elif "rsw" in data.data:
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(set_rsw_demo_cmd)
            time.sleep(0.1)
        

    def record_thread(self):
        global recordflag, recordbuff
        recordflag = 1
        recordbuff = ''
        recordname = time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.txt'
        fd = open(recordname, 'w+')
        self.get_logger().info('begin recording file name is {}'.format(recordname))
        while recordflag:
            if len(recordbuff):
                fd.write(recordbuff)
                recordbuff = ''
            else:
                time.sleep(1)
        fd.close()
        self.get_logger().info("stop recording")

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    
    try:
        imu_node.wt_imu = serial.Serial(port=imu_node.port, baudrate=imu_node.baudrate, timeout=10)
        if imu_node.wt_imu.isOpen():
            imu_node.get_logger().info("\033[32mSerial port enabled successfully...\033[0m")
        else:
            imu_node.wt_imu.open()
            imu_node.get_logger().info("\033[32mSerial port enabled successfully...\033[0m")
    except Exception as e:
        imu_node.get_logger().info(e)
        imu_node.get_logger().info("\033[31mFailed to open the serial port\033[0m")
        exit(0)
    
    while rclpy.ok():
        try:
            buff_count = imu_node.wt_imu.inWaiting()
            if buff_count > 0 and iapflag == 0:
                buff_data = imu_node.wt_imu.read(buff_count)
                imu_node.get_logger().info(buff_data)
                if recordflag:
                    recordbuff.extend(buff_data)
                for i in range(0, buff_count):
                    imu_node.handle_serial_data(buff_data[i])
        except Exception as e:
            imu_node.get_logger().info("exception:" + str(e))
            imu_node.get_logger().info("imu loss of connection, poor contact, or broken wire")
            exit(0)
    rclpy.spin(imu_node)
  
    # rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
