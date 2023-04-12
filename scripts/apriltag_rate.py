#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from tf2_msgs.msg import TFMessage

time_prev = time.time()
x_prev = 0
x = 0

def call_tf(msg):
    global time_prev
    global x_prev
    
    x = msg.transforms[0].transform.translation.x

    if x_prev != x:
        dt = time.time() - time_prev
        time_prev = time.time()
        
        print('Time :')
        print(dt)
        print('hz : ')
        print(1/dt)
        print()

        x_prev = x


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, call_tf)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
