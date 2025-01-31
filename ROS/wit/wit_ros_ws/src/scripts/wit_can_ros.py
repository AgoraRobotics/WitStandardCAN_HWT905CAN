#!/usr/bin/env python3
import serial
import struct
import rclpy
import time
import math
import sys
import platform
import threading
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler


readreg = 0
key = 0
flag = 0
iapflag = 0
global recordflag
buff = {}
calibuff = list()
global recordbuff
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
mag_offset = [0, 0, 0]
mag_range = [0, 0, 0]
global wt_imu
baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

# Find ttyUSB* devices
def find_ttyUSB(self):
    self.get_logger().info('The default serial port of the imu is /dev/ttyUSB0, if multiple serial port devices are identified, modify the serial port corresponding to the imu in the launch file')
    
    posts = [port.device for port in serial.tools.list_ports.comports()
             if 'USB' in port.device]
    
    self.get_logger().info('There are {} {} serial port devices connected to the current PC: {}'.format(
        len(posts), 'USB', posts))

# Convert hexadecimal to ieee floating point number
def hex_to_short(raw_data):
    return list(struct.unpack("hhh", bytearray(raw_data)))

# Process serial port data
def handleSerialData(self, raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, readreg, calibuff, flag, mag_offset, mag_range
    global angle_tip
    angle_flag = False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 8:  #According to the judgment of the data length bit, obtain the corresponding length data
        return
    else:
        data_buff = list(buff.values())  # Get all values in the dictionary
        if buff[1] == 0x51:
            acceleration = [hex_to_short(data_buff[2:8])[
                i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]

        elif buff[1] == 0x52:
            angularVelocity = [hex_to_short(data_buff[2:8])[
                i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

        elif buff[1] == 0x53:
            temp = hex_to_short(data_buff[2:8])
            angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]

            angle_flag = True

        elif buff[1] == 0x54:
            magnetometer = hex_to_short(data_buff[2:8])
            if flag:
                calibuff.append(magnetometer[0:2])

        elif buff[1] == 0x5f:

            readval = hex_to_short(data_buff[2:8])
            if readreg == 0x0b:
                mag_offset = readval
            else:
                mag_range = readval

            self.get_logger().info(readval)

        else:
            # print("This data processing class does not provide the parsing of " + str(buff[1]) + "")
            # print("or data error")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag == True:

            stamp = self.get_clock().now().to_msg()

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_link"

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = "base_link"

            angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(
                angle_radian[0], angle_radian[1], angle_radian[2])

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

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

            angle_tip = 0


def recordThread(self):
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
    set_rsw_demo_cmd = b'\xff\xaa\x02\x1f\x00'  # output time acc gyro angle mag

    self.get_logger().info('callback')
    self.get_logger().info(data)
    if "mag" in data.data:
        wt_imu.write(unlock_imu_cmd)
        time.sleep(0.1)
        wt_imu.write(reset_magx_offset_cmd)
        time.sleep(0.1)
        wt_imu.write(reset_magy_offset_cmd)
        time.sleep(0.1)
        wt_imu.write(reset_magz_offset_cmd)
        time.sleep(0.1)
        wt_imu.write(reset_mag_param_cmd)
        time.sleep(0.1)
        wt_imu.write(enter_mag_cali_cmd)
        time.sleep(0.1)
        flag = 1
        calibuff = []
        mag_offset = [0, 0, 0]
        mag_range = [500, 500, 500]
    elif "exti" in data.data:
        flag = 0
        wt_imu.write(unlock_imu_cmd)
        time.sleep(0.1)
        wt_imu.write(exti_cali_cmd)
        time.sleep(0.1)
        wt_imu.write(save_param_cmd)
        time.sleep(1)
        readreg = 0x0b
        wt_imu.write(read_mag_offset_cmd)
        time.sleep(1)
        readreg = 0x1c
        wt_imu.write(read_mag_range_cmd)
        time.sleep(1)
        datalen = len(calibuff)
        self.get_logger().info('cali data {}'.format(datalen))
        r = list()
        if datalen > 0:
            for i in range(datalen):
                tempx = ((calibuff[i][0] - mag_offset[0])
                         * 2/float(mag_range[0]))
                tempy = ((calibuff[i][1] - mag_offset[1])
                         * 2/float(mag_range[1]))
                temp = tempx*tempx+tempy*tempy-1
                r.append(abs(temp))
            sumval = sum(r)
            r_n = float(sumval)/datalen
            if r_n < 0.05:
                self.get_logger().info('magnetic field calibration results are very good')
            elif r_n < 0.1:
                self.get_logger().info('magnetic field calibration results are good')
            else:
                self.get_logger().info('magnetic field calibration results is bad, please try again')
    elif "version" in data.data:
        self.get_logger().info('sensor version is {}'.format(version))
    elif "begin" in data.data:
        record_thread = threading.Thread(target=recordThread)
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
                    wt_imu.write(unlock_imu_cmd)
                    time.sleep(0.1)
                    wt_imu.write(cmd)
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
                    wt_imu.write(unlock_imu_cmd)
                    time.sleep(0.1)
                    wt_imu.write(cmd)
                    time.sleep(0.1)
                    wt_imu.baudrate = baud
        except Exception as e:
            self.get_logger().info(e)
    elif "rsw" in data.data:
        wt_imu.write(unlock_imu_cmd)
        time.sleep(0.1)
        wt_imu.write(set_rsw_demo_cmd)
        time.sleep(0.1)


def thread_job(self):
    self.get_logger().info("thread run")
    rclpy.spin_once(node)


def AutoScanSensor(self):
    global wt_imu, baudlist
    try:
        for baud in baudlist:
            read_cmd = '\xff\xaa\x27\x00\x00'.encode("utf-8")
            wt_imu.baudrate = baud
            wt_imu.flushInput()
            wt_imu.write(read_cmd)
            time.sleep(0.2)
            buff_count = wt_imu.inWaiting()
            if buff_count >= 11:
                buff_data = wt_imu.read(buff_count)
                val = bytearray(buff_data)
                for i in range(len(val)):
                    if val[i] == 0x55:
                        sumval = sum(val[i:i+10])
                        if sumval == val[i+10]:
                            self.get_logger().info('{} baud find sensor'.format(baud))
                            return

    except Exception as e:
        self.get_logger().info("exception:" + str(e))
        self.get_logger().info("imu loss of connection, poor contact, or broken wire")
        exit(0)


if __name__ == "__main__":
    
    recordflag = 0
    recordbuff = list()
    wt_imu = serial.Serial()
    python_version = platform.python_version()[0]

    rclpy.init()
    node = rclpy.create_node('imu')
    find_ttyUSB(self=node)
    
    node.declare_parameter("port", "/dev/ttyUSB0")
    node.declare_parameter("baud", 230400)
    
    port = node.get_parameter("port").value
    baudrate = node.get_parameter("baud").value
    
    node.get_logger().info("IMU Type: HCAN Port:%s baud:%d" % (port, baudrate))
    
    imu_msg = Imu()
    mag_msg = MagneticField()
    
    subscription = node.create_subscription(String, '/wit/cali', self.callback, 10)
    
    add_thread = threading.Thread(target=thread_job(self=node))
    add_thread.start()
    
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=10)
        if wt_imu.isOpen():
            node.get_logger().info("\033[32mSerial port enabled successfully...\033[0m")
        else:
            wt_imu.open()
            node.get_logger().info("\033[32mSerial port enabled successfully...\033[0m")
    except Exception as e:
        node.get_logger().info(e)
        node.get_logger().info("\033[31mFailed to open the serial port\033[0m")
        exit(0)
    else:
        # AutoScanSensor()
        imu_pub = node.create_publisher(Imu, 'wit/imu', 10)
        mag_pub = node.create_publisher(MagneticField, 'wit/mag', 10)

        while rclpy.ok():
            try:
                buff_count = wt_imu.inWaiting()

                if buff_count > 0 and iapflag == 0:

                    buff_data = wt_imu.read(buff_count)

                    if recordflag:
                        recordbuff = recordbuff + buff_data
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
            except Exception as e:
                node.get_logger().info("exception:" + str(e))
                node.get_logger().info("imu loss of connection, poor contact, or broken wire")
                exit(0)
    rclpy.shutdown()
