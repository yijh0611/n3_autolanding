#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# 한글 주석 테스트

import rospy
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO
import time

def callback(data): # control gimbal
    print(data)
    ang1, ang2 = data.data
    
    
    print()
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", ang1)
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", ang2)
    
    global pwm
    global pwm2

    pwm.ChangeDutyCycle(ang1/10)
    pwm2.ChangeDutyCycle(ang2/10)
    # time.sleep(0.05)

# # Servo init(전역변수 느낌)
SERVO_PIN = 33
SERVO_PIN2= 32
channels = [SERVO_PIN, SERVO_PIN2] # 나중에 추가

GPIO.setmode(GPIO.BOARD)
GPIO.setup(channels, GPIO.OUT, initial=GPIO.HIGH)

pwm = GPIO.PWM(SERVO_PIN, 50) # 50hz
pwm2 = GPIO.PWM(SERVO_PIN2, 50)

pwm.start(5)
pwm2.start(5)

pwm.ChangeDutyCycle(80/10)
pwm2.ChangeDutyCycle(86/10)
time.sleep(0.1)
# print(70)

if __name__ == '__main__':
    rospy.init_node('gimbal_control', anonymous=True)
    rospy.Subscriber("gimbal_control", Float64MultiArray, callback)
    print('start gimbal control node')
    
    ## Test ##
    
    ## test until here ##
    
    # # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()