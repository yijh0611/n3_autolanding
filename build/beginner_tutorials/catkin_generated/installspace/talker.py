#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # pub_img = rospy.Publisher('camera_rect/image_rect',Image,queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
def make_camera_msg(cam):
    camera_info_msg = CameraInfo()
    width, height = cam[0], cam[1]
    fx, fy = cam[2], cam[3]
    cx, cy = cam[4], cam[5]
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
                         
    camera_info_msg.D = [0, 0, 0, 0]
    
    camera_info_msg.P = [fx, 0, cx, 0,
                         0, fy, cy, 0,
                         0, 0, 1, 0]
    return camera_info_msg   
    
def make_camera_msg2():
    camera_info_msg = CameraInfo()
    width, height = 640, 480
    fx, fy = 1.638612, 2.963136
    cx, cy = 320, 240
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
                         
    camera_info_msg.D = [0, 0, 0, 0]
    
    camera_info_msg.P = [fx, 0, cx, 0,
                         0, fy, cy, 0,
                         0, 0, 1, 0]
    return camera_info_msg

if __name__ == '__main__':
    try:
        # 1 : Saved image; 0 : webcam
        img_tst = 1
        if img_tst == 1:
            # For image(test)
            src = '/home/aims/AprilTag_08.png'
            src = '/home/aims/Downloads/Tag_example.png'
            # capture = cv2.imread('/home/aims/AprilTag_08.png')
        else:
        
            # For camera
            capture = cv2.VideoCapture(0)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        camera_info_msg = make_camera_msg2()
        
        bridge = CvBridge()
        
        pub_img = rospy.Publisher('camera_rect/image_rect',Image,queue_size=10)
        pub_cam_info = rospy.Publisher('camera_rect/camera_info',CameraInfo,queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(60) # 60hz
        
        time_prev = time.time()
        i = 0
        print(i)
        while cv2.waitKey(33) < 0: # cv2.waitKey(33) < 0 and not rospy.is_shutdown()
            if img_tst == 1:
                frame = cv2.imread(src)
                if i > 10 :
                    frame = cv2.bitwise_not(frame)
                cv2.imshow("VideoFrame", frame)
            else:
                ret, frame = capture.read()
                cv2.imshow("VideoFrame", frame)
            # rospy.loginfo(frame)
            

            # img_bridge = bridge.cv2_to_imgmsg(frame, "bgr8")
            # pub_img.publish(img_bridge)
            pub_cam_info.publish(camera_info_msg)
            if time.time() > time_prev + 1:
                print(i,rospy.get_time())
                i = 0
                time_prev = time.time()
            i = i + 1
            # print(i)
            rate.sleep()
        if img_tst == 1:
            capture.release()
        # cv2.destroyAllWindows()
        #talker()
    except rospy.ROSInterruptException:
        pass
        
