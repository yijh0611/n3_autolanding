#!/usr/bin/env python
# # -*- coding: utf-8 -*
import rospy
import numpy as np
import math
import threading

# 라이다
import serial
import time

# Lidar Publish
from std_msgs.msg import Float64MultiArray

distance = 0
strength = 0
def getTFminiData():
    count = ser.in_waiting
    global distance, strength

    if count > 8:
        recv = ser.read(9)  
        ser.reset_input_buffer()
        # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
        # type(recv[0]), 'str' in python2, 'int' in python3
        
        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            print('(', distance, ',', strength, ')')
            
            ser.reset_input_buffer()
            # return distance, strength
            
        if recv[0] == 'Y' and recv[1] == 'Y':     #python2 - ros는 python2
            # print('Python 2')
            lowD = int(recv[2].encode('hex'), 16)      
            highD = int(recv[3].encode('hex'), 16)
            lowS = int(recv[4].encode('hex'), 16)      
            highS = int(recv[5].encode('hex'), 16)
            distance = lowD + highD * 256
            strength = lowS + highS * 256
            # print(distance, strength)
            # return distance, strength
    # else:
    #     print('else')
    #     return 0,0

def is_interrupt_thread():
    global is_interrupt

    try:
        pass

    except KeyboardInterrupt:
        for i in range(10):
            print('End')
        is_interrupt = True

is_interrupt = False

if __name__ == '__main__':
    rospy.init_node('lidar_ros', anonymous=True)

    # Publish
    pub_lidar = rospy.Publisher('lidar', Float64MultiArray, queue_size=10)

    # 종료 쓰레드
    t = threading.Thread(target=is_interrupt_thread)
    # t.daemon = True
    t.start()

    # 라이다
    # ser = serial.Serial("/dev/ttyTHS1", 115200)
    
    try: # N3보다 나중에 연결하면 1번
        ser = serial.Serial("/dev/ttyUSB1", 115200) # 9600
    except: # N3보다 먼저 연결하면 0번
        ser = serial.Serial("/dev/ttyUSB0", 115200) # 9600

    try:
        if ser.is_open == False:
            ser.open()
        while not rospy.is_shutdown():
            getTFminiData()

            data_pub = Float64MultiArray()
            data_pub.data = np.array([distance, strength])
            pub_lidar.publish(data_pub)

            if is_interrupt:
                break

    except KeyboardInterrupt:   # Ctrl+C
        is_interrupt = True
        if ser != None:
            ser.close()
        sys.exit()




# # !/usr/bin/env python3.6
# # -*- coding: utf-8 -*-

# # 코드 특징
# # pmw 값을 수신해서 Publish한다. 만약에 실행이 안된다면, 그냥 따로 실행해서 알아서 넘파이로 저장하는거로 바꾸기
# import rospy
# import numpy as np
# import math
# import time
# import threading
# # PMW3901 part
# import argparse
# from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

# # # Velocity Publish
# from std_msgs.msg import Float64MultiArray
# # vo position
# from dji_sdk.msg import VOPosition

# # PMW3901
# parser = argparse.ArgumentParser()
# parser.add_argument('--rotation', type=int,
#                     default=0, choices=[0, 90, 180, 270],
#                     help='Rotation of sensor in degrees.')
# parser.add_argument('--spi-slot', type=str,
#                     default='front', choices=['front', 'back'],
#                     help='Breakout Garden SPI slot.')

# args = parser.parse_args()

# if args.spi_slot == 'front' :
#     flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_FRONT_BCM)
# else:
#     flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_BACK_BCM)
# # flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_FRONT_BCM if args.spi_slot == 'front' else BG_CS_BACK_BCM)
# flo.set_rotation(args.rotation)

# tx = 0
# ty = 0

# time_prev = time.time()

# is_interrupt = False

# def is_interrupt_thread():
#     global is_interrupt

#     try:
#         pass

#     except KeyboardInterrupt:
#         for i in range(10):
#             print('End')
#         is_interrupt = True


# # def ros_spin_thread():
# #     print("Ros spin thread start")
# #     rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('pmw_3901_ros', anonymous=True)
#     print('Start PMW3901 node')

#     # Publish
#     pub_pmw3901 = rospy.Publisher('pmw3901', Float64MultiArray, queue_size=10)
    
#     # 종료 쓰레드
#     t = threading.Thread(target=is_interrupt_thread)
#     # t.daemon = True
#     t.start()

#     # 무한루프 실행
#     try:
#         while not rospy.is_shutdown():
#             # print('While True')
#             try:
#                 x, y = flo.get_motion()
#             except RuntimeError:
#                 # print('except')
#                 continue

#             tx += x
#             ty += y
#             print("Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(x, y, tx, ty))
            
#             data_pub = Float64MultiArray()
#             data_pub.data = np.array([x, y, tx, ty])
#             pub_pmw3901.publish(data_pub)

#             time.sleep(0.01)

#             if is_interrupt:
#                 break

#     except KeyboardInterrupt:
#         is_interrupt = True
#         sys.exit()