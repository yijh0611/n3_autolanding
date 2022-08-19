#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

print('start')

SERVO_PIN = 33
SERVO_PIN2= 32
channels = [SERVO_PIN, SERVO_PIN2] # 나중에 추가

GPIO.setmode(GPIO.BOARD)
GPIO.setup(channels, GPIO.OUT, initial=GPIO.HIGH)

pwm = GPIO.PWM(SERVO_PIN, 50) # 50hz
pwm2 = GPIO.PWM(SERVO_PIN2, 50)

pwm.start(5)
pwm2.start(5)
time.sleep(1)

# i = 57
# print(i)
# pwm2.ChangeDutyCycle(87/10)
# pwm.ChangeDutyCycle(i/10) # ???
# time.sleep(1)

# i = 80
# print(i)
# pwm.ChangeDutyCycle(i/10)
# time.sleep(2)

i = 57
print(i)
pwm.ChangeDutyCycle(77/10)
pwm2.ChangeDutyCycle(i/10) # ???
time.sleep(1)

i = 87
print(i)
pwm2.ChangeDutyCycle(i/10)
time.sleep(2)

print('end')

# j = 0
# while(True):
#     for i in range(56,77):
#         if(i%10==0):
#             print(i)
#         pwm.ChangeDutyCycle(i/10)
#         pwm2.ChangeDutyCycle(i/10)
#         time.sleep(0.5)
#         if i == 75:
#             print('sleep for 5 sec')
#             time.sleep(2)
#     j = j + 1
#     print(f'{j} times')
        