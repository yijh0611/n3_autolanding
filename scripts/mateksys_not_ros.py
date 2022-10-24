# -*- coding: utf-8 -*-
import sys
import time
import struct
import threading
import serial
from serial.tools import list_ports
import pandas as pd
import numpy as np

ports = list_ports.comports()
devices = [info.device for info in ports]
if len(devices) == 0:
        print("error: device not found")
else:
    for i_for in range(len(devices)):
        print("found: device %s" % devices[i_for])
    print(devices[0])
# comdev = serial.Serial(devices[0], baudrate=115200, parity=serial.PARITY_NONE)
comdev = serial.Serial(devices[0], baudrate=115200, parity=serial.PARITY_NONE)
#comdev = serial.Serial('COM1', baudrate=115200, parity=serial.PARITY_NONE)

distance = 0
vx = 0
vy = 0
time_start = time.time()

time_np = np.array([])
vx_np = np.array([])
vy_np = np.array([])
distance_np = np.array([])

def rev_thread_func():
    while not rcv_thread_stop.is_set():
        # print("while start")
        # if comdev.readable():
        #     print("readable")
        # else:
        #     print("not readable")
        # try:
        #     bytesA = comdev.read()
        # except:
        #     print("Error")
        #     continue
        bytesA = comdev.read(1)
        # print('Comdev read')
        # print(bytesA)
        mark = int.from_bytes(bytesA, 'little')
        if mark == 36:
            # print("mark 36")
            bytesB = comdev.read(5)
            bytesC = comdev.read(2)
            psize = int.from_bytes(bytesC, 'little')
            bytesD = comdev.read(psize)
            bytesE = comdev.read(1)
            if psize == 5:
                val1 = int.from_bytes(bytesD[0:1], 'little')
                val2 = int.from_bytes(bytesD[1:5], 'little', signed=True)
                # print()
                # print("Distance : ",val2)
                distance = val2
                # print(distance)
                val3 = 0
                tabs = '\t\t'
            elif psize == 9:
                val1 = int.from_bytes(bytesD[0:1], 'little')
                val2 = int.from_bytes(bytesD[1:5], 'little', signed=True)
                # print("X : ", val2)
                vx = int(distance * val2) / 1000
                
                val3 = int.from_bytes(bytesD[5:8], 'little', signed=True)
                # print("Y : ", val3)
                vy = int(distance * val3) / 1000
                if distance != -1:
                    print("vx : ", vx)
                    print("vy : ", vy)
                    global time_np, vx_np, vy_np, distance_np
                    time_np = np.append(time_np, time.time() - time_start)
                    vx_np = np.append(vx_np, vx)
                    vy_np = np.append(vy_np, vy)
                    distance_np = np.append(distance_np, distance)
                    data = {'time': time_np,
                            'distance' : distance_np,
                            'vx': vx_np,
                            'vy': vy_np}
                    df = pd.DataFrame(data)
                    df.to_csv('matek_test.csv')
                    for i_for in range(5):
                        print()
                tabs = '\t'

                # time.sleep(0.5)
            else:
                continue
            # viewtext1 = str(bytesA.hex()) + ' : ' + str(bytesB.hex()) + ' : ' + str(bytesC.hex()) + ' : ' + str(bytesD.hex()) + ' : ' + str(bytesE.hex())
            # # viewtext1 = str(bytesA.hex()) + ' : ' + str(bytesB.hex()) + ' : ' + str(bytesC.hex()) + ' : ' + str(bytesD.hex()) + ' : ' + str(bytesE.hex())
            # viewtext2 = '(' + str(val1) + ',' + str(val2) + ',' + str(val3) + ')'
            # print(viewtext1, tabs, viewtext2)
    comdev.close()
    print("comdev close")
    return

rcv_thread_stop = threading.Event()
rcv_thread = threading.Thread(target = rev_thread_func)
rcv_thread.daemon = True
rcv_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    rcv_thread_stop.is_set()
    rcv_thread.join(1)
    print("done")