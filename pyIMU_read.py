#!/usr/bin/env python3
"""
pyserial IMU read for EBIMU24GV5_rev11
===============================================

##################
# pyIMU_read.py
# Sunhong Kim
# tjsghd101@naver.com
# 28. Aug. 2023
##################
"""

import sys
from numpy import pi, cos, sin, tan, square
from casadi import vertcat, horzcat, sumsqr, arctan2
from casadi import pi, cos, sin
import copy
import os
import numpy as np

from multiprocessing import Process, Manager
import rospy
import tf
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
import serial,time



def imu_run():

    rospy.init_node('cona_imu', anonymous=True)


    pub = rospy.Publisher('/cona_imu', Accel, queue_size=50)

    imu_msg = Accel()
    imu_frq = 100
    
    base_dist = 0.2

    rate = rospy.Rate(imu_frq)

    #initialization and open the port
    ser = serial.Serial()
    ser.port = "/dev/ttyUSB4"
    ser.baudrate = 921600
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check: no parity
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        #ser.timeout = None          #block read
    ser.timeout = 5               #non-block read
        #ser.timeout = 2              #timeout block read
    ser.xonxoff = False     #disable software flow control
    ser.rtscts = False     #disable hardware (RTS/CTS) flow control
    ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control

    ser.open()

    while not rospy.is_shutdown():

        init_time=time.time()

        response1 = ser.readline()
        response2 = ser.readline()
        # print(len(response1), len(response2))

        # Decode bytes to string and split data
        splitted_data1 = response1.decode('utf-8').strip().split(',')
        splitted_data2 = response2.decode('utf-8').strip().split(',')

        # Convert to float and store in array
        if splitted_data1[0] == '100-1':                        # At the corner
            id01_acc = [float(x) for x in splitted_data1[5:8]]
            # print("Data as float array for 100-1: \n", id01_acc)
        else:
            id01_acc=[0,0,0]

        # Convert to float and store in array
        if splitted_data2[0] == '100-2':                        # At the center
            id02_acc = [float(x) for x in splitted_data2[5:8]]
            # print("Data as float array for 100-2: \n", id02_acc)
        else:
            id02_acc=[0,0,0]
           
        imu_msg.linear.x=id02_acc[0]*9.8
        imu_msg.linear.y=id02_acc[1]*9.8
        imu_msg.linear.z=id02_acc[2]*9.8
        
        imu_msg.angular.x = 0
        imu_msg.angular.y = 0
        imu_msg.angular.z = id01_acc[0]*9.8/base_dist

        pub.publish(imu_msg)

        rate.sleep()



#############################################################################################################################

if __name__=='__main__':
    imu_task = Process(target=imu_run, args=())
    try:
        imu_task.start()

        imu_task.join()

    except KeyboardInterrupt:
        imu_task.terminate()
