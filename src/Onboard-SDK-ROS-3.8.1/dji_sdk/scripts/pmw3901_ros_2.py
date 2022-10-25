#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

# 코드 특징
# pmw 값을 수신해서 Publish한다. 만약에 실행이 안된다면, 그냥 따로 실행해서 알아서 넘파이로 저장하는거로 바꾸기
import rospy
import numpy as np
import math
import time
import threading
# PMW3901 part
import argparse
from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

# # Velocity Publish
from std_msgs.msg import Float64MultiArray
# vo position
from dji_sdk.msg import VOPosition

# PMW3901
parser = argparse.ArgumentParser()
parser.add_argument('--rotation', type=int,
                    default=0, choices=[0, 90, 180, 270],
                    help='Rotation of sensor in degrees.')
parser.add_argument('--spi-slot', type=str,
                    default='front', choices=['front', 'back'],
                    help='Breakout Garden SPI slot.')

args = parser.parse_args()

if args.spi_slot == 'front' :
    flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_FRONT_BCM)
else:
    flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_BACK_BCM)
# flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_FRONT_BCM if args.spi_slot == 'front' else BG_CS_BACK_BCM)
flo.set_rotation(args.rotation)

tx = 0
ty = 0

time_prev = time.time()

is_interrupt = False

def is_interrupt_thread():
    global is_interrupt

    try:
        pass

    except KeyboardInterrupt:
        for i in range(10):
            print('End')
        is_interrupt = True


# def ros_spin_thread():
#     print("Ros spin thread start")
#     rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pmw_3901_ros', anonymous=True)
    print('Start PMW3901 node')

    # Publish
    pub_pmw3901 = rospy.Publisher('pmw3901', Float64MultiArray, queue_size=10)
    
    # 종료 쓰레드
    t = threading.Thread(target=is_interrupt_thread)
    # t.daemon = True
    t.start()

    # 무한루프 실행
    try:
        while not rospy.is_shutdown():
            # print('While True')
            try:
                x, y = flo.get_motion()
            except RuntimeError:
                # print('except')
                continue

            tx += x
            ty += y
            print("Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(x, y, tx, ty))
            
            data_pub = Float64MultiArray()
            data_pub.data = np.array([x, y, tx, ty])
            pub_pmw3901.publish(data_pub)

            time.sleep(0.01)

            if is_interrupt:
                break

    except KeyboardInterrupt:
        is_interrupt = True
        sys.exit()