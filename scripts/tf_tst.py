#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 한국어 주석 적기 위함

import rospy
# 칼만필터 부분
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import math
import time
import threading

# # AprilTag
from tf2_msgs.msg import TFMessage

# # Velocity Publish
from std_msgs.msg import Float64MultiArray

tf = 0
def callback_tf(tf_data):
    global tf
    tf = tf_data.transforms[0].transform.translation # x,y,z
    # print(123)
    # print("tf signal", time.time())

    '''
    transforms: 
        - 
            header: 
            seq: 0
            stamp: 
                secs: 0
                nsecs:         0
            frame_id: ''
            child_frame_id: "tag_6"
            transform: 
                translation: 
                    x: 0.105127976721
                    y: 0.00396826638588
                    z: 0.440499491408
                rotation: 
                    x: 0.122698219354
                    y: 0.932748650317
                    z: 0.198426097724
                    w: -0.274867579102
    '''

def ros_spin_thread():
    print("Ros spin thread start")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tf_tst', anonymous=True)
    print('Start Tf singal count')

    # tf 정보 수신
    rospy.Subscriber("tf", TFMessage, callback_tf)
    
    # 멀티쓰레드 만들어서 센서 값 받은거 계산해서 출력하게 만들기
    t = threading.Thread(target=ros_spin_thread)
    t.daemon = True
    t.start()
    
    while tf == 0:
        pass
    
    time_count = time.time()
    tf_count = 0
    tf_prev = 0
    print('Loop start')

    while True:
        if tf_prev != tf:
            tf_count = tf_count + 1
            tf_prev = tf

        if time_count + 1 < time.time():
            print("Tag per second : ", tf_count)
            tf_count = 0
            time_count = time.time()