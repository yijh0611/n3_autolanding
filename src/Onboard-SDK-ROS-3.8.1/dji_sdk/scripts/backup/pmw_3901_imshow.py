#!/usr/bin/env python
import time
import argparse
from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM
import numpy as np

print("""frame_capture.py - Capture the raw frame data from the PMW3901
Press Ctrl+C to exit!
""")

parser = argparse.ArgumentParser()
parser.add_argument('--rotation', type=int,
                    default=0, choices=[0, 90, 180, 270],
                    help='Rotation of sensor in degrees.')
parser.add_argument('--spi-slot', type=str,
                    default='front', choices=['front', 'back'],
                    help='Breakout Garden SPI slot.')

args = parser.parse_args()

# print(args.spi_slot)
flo = PMW3901(spi_port=0, spi_cs=0, spi_cs_gpio=BG_CS_FRONT_BCM if args.spi_slot == 'front' else BG_CS_BACK_BCM)
flo.set_rotation(args.rotation)

def value_to_char(value):
    charmap = [" ", "░", "▒", "▓", "█"]
    value /= 255
    value *= len(charmap) - 1
    value = int(value)
    return charmap[value] * 2  # Double chars to - sort of - correct aspect ratio


while True:
    try:
        print("Capturing...")
        data = flo.frame_capture()
        for y in range(35):
            y = 35 - y - 1 if args.rotation in (180, 270) else y
            for x in range(35):
                x = 35 - x - 1 if args.rotation in (180, 90) else x
                if args.rotation in (90, 270):
                    offset = (x * 35) + y
                else:
                    offset = (y * 35) + x
                value = data[offset]
                print(value_to_char(value), end="")
            print("")
        print("2...")
        time.sleep(1.0)
        print("Get Ready!")
        time.sleep(1.0)

    except KeyboardInterrupt:
        # print("Stop")
        pass

# try:
#     while True:
#         print("Capturing...")
#         data = flo.frame_capture()
#         for y in range(35):
#             y = 35 - y - 1 if args.rotation in (180, 270) else y
#             for x in range(35):
#                 x = 35 - x - 1 if args.rotation in (180, 90) else x
#                 if args.rotation in (90, 270):
#                     offset = (x * 35) + y
#                 else:
#                     offset = (y * 35) + x
#                 value = data[offset]
#                 print(value_to_char(value), end="")
#             print("")
#         print("5...")
#         time.sleep(1.0)
#         print("4...")
#         time.sleep(1.0)
#         print("3...")
#         time.sleep(1.0)
#         print("2...")
#         time.sleep(1.0)
#         print("Get Ready!")
#         time.sleep(1.0)

# except KeyboardInterrupt:
#     pass