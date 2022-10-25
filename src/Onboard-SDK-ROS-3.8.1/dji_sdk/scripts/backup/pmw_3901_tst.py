#!/usr/bin/env python
import time
import argparse
from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

print("""motion.py - Detect flow/motion in front of the PMW3901 sensor.
Press Ctrl+C to exit!
""")
# for i in range(10):
#     print('Start')

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

tx = 0
ty = 0

time_prev = time.time()
try:
    while True:
        # print('While True')
        try:
            x, y = flo.get_motion()
        except RuntimeError:
            print('except')
            continue
        tx += x
        ty += y
        print("Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(x, y, tx, ty))
        time.sleep(0.01)
        # print(time.time() - time_prev)
        # time_prev = time.time()
except KeyboardInterrupt:
    pass