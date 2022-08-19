#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 한글 주석 테스트

import rospy
from std_msgs.msg import Float64MultiArray
import Jetson.GPIO as GPIO
import time

def gimb(t_high):
        pwm.ChangeDutyCycle(t_high/100.0)   
        pwm2.ChangeDutyCycle(t_high/100.0)
        time.sleep(1)

def callback(data): # control gimbal
    ang1, ang2 = data.data
    
    print()
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", ang1)
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", ang2)
    
    ang1 = ang1/2 + 50
    ang2 = ang2/2 + 50
        
    # ###### edit under this line #####
    SERVO_PIN = 33
    SERVO_PIN2= 32
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN, GPIO.OUT)

    hz = 50 # 50
    pwm = GPIO.PWM(SERVO_PIN, hz)
    pwm2 = GPIO.PWM(SERVO_PIN2, hz)
    pwm.start(3.0)
    pwm2.start(3.0)

    # print('move gimbal')
    pwm.ChangeDutyCycle(ang1/10) #
    pwm2.ChangeDutyCycle(ang2/10)
    time.sleep(0.1)
    
def test(ang):
    # SERVO_PIN = 33
    # SERVO_PIN2= 32
    # pwm = GPIO.PWM(SERVO_PIN, 50)
    # pwm2 = GPIO.PWM(SERVO_PIN2, 50)
    # pwm.start(3.0)
    # pwm2.start(3.0)
    
    global pwm
    global pwm2

    # print('move gimbal')
    pwm.ChangeDutyCycle(ang/10) #
    pwm2.ChangeDutyCycle(ang/10)
    print(ang)
    time.sleep(0.1)

if __name__ == '__main__':
    
    
    rospy.init_node('gimbal_control', anonymous=True)
    rospy.Subscriber("gimbal_control", Float64MultiArray, callback)
    print('start gimbal control node')
    
    
    # for i in range(2):
    ############################
    SERVO_PIN = 33
    SERVO_PIN2= 32
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    
    pwm = GPIO.PWM(SERVO_PIN, 50)
    pwm2 = GPIO.PWM(SERVO_PIN2, 50)
    
    pwm.start(3.0)
    pwm2.start(3.0)
    time.sleep(0.5)
    
    # high
    t_high = 97
    pwm.ChangeDutyCycle(77/10.0)
    pwm2.ChangeDutyCycle(t_high/10.0)
    time.sleep(0.5)
    
    # # low
    t_high = 57
    pwm.ChangeDutyCycle(56/10.0)
    pwm2.ChangeDutyCycle(t_high/10.0)
    time.sleep(3)

    # print(t_high)
    # print('????')
    # time.sleep(5)
    
#     for t_high in range(50, 100): # 30~125
#         pwm.ChangeDutyCycle(t_high/10.0)
#         pwm2.ChangeDutyCycle(t_high/10.0)
#         print(t_high)
#         time.sleep(0.1)
#     time.sleep(3)
    
#     pwm.ChangeDutyCycle(50/10.0)
#     pwm2.ChangeDutyCycle(50/10.0)
#     print(11111)
#     time.sleep(5)
    
#     pwm.ChangeDutyCycle(100/10.0)
#     pwm2.ChangeDutyCycle(100/10.0)
#     print(22222)
#     time.sleep(3)
    
        
#     # pwm.ChangeDutyCycle(3.0)
#     # pwm2.ChangeDutyCycle(3.0)
#     # time.sleep(1.0)
#     # pwm.ChangeDutyCycle(0.0)
#     # pwm2.ChangeDutyCycle(3.0)

#     pwm.stop()
#     GPIO.cleanup()

#     # # print('move gimbal')
#     # pwm.ChangeDutyCycle(50) # %
#     # pwm2.ChangeDutyCycle(0)
#     # time.sleep(2)
    
#     # pwm.ChangeDutyCycle(0) # %
#     # pwm2.ChangeDutyCycle(50)
#     # time.sleep(2)
    

#     # pwm.stop()
#     # pwm2.stop()
#     # GPIO.cleanup()
#     # print('i',i)
#     # time.sleep(3)
    #     ############################
        
    # # SERVO_PIN = 33
    # # SERVO_PIN2= 32
    
    # # GPIO.setwarnings(False)
    # # GPIO.setmode(GPIO.BOARD)
    # # GPIO.setup(SERVO_PIN, GPIO.OUT)

    # # pwm = GPIO.PWM(SERVO_PIN, 50)
    # # pwm.start(3.0)
    # # pwm2 = GPIO.PWM(SERVO_PIN2, 50)
    # # pwm2.start(3.0)
    
    # spin() simply keeps python from exiting until this node is stopped
    
    
    rospy.spin()