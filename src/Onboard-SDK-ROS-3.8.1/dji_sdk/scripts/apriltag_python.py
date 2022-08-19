#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 한국어 주석 적기 위함

# 작동은 하는데, FPS가 느려서 사용하기 힘들다.

import cv2
import numpy as np
from dt_apriltags import Detector
import time

cap = cv2.VideoCapture(0)
# cap.set(3, 640) # 가로
# cap.set(4, 480) # 세로
# cap.set(cv2.CAP_PROP_FPS, 120)

fps = cap.get(cv2.CAP_PROP_FPS)
# print('fps', fps)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap.set(cv2.CAP_PROP_FPS, 100)
print('fps', fps)
time.sleep(2)
# # C++
# VideoCapture cap(0);
# cap.set(CAP_PROP_FRAME_WIDTH, 640); // 1280
# cap.set(CAP_PROP_FRAME_HEIGHT, 480); // 720
# cap.set(CAP_PROP_FPS, 120);
# double fps = cap.get(CAP_PROP_FPS);
# cout << fps << "fps" << endl;

at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

camera_params = (501.92767418, 500.12588919, 331.22594614, 225.79436303)

count = 0
count_tag = 0
start_time = time.time()
while True:
    check, frame = cap.read()
 
    # cv2.imshow('video', frame)
    print(frame.shape)

    # frame = np.array(frame)

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # print(len(frame.shape))
    tags = at_detector.detect(frame_gray, True, camera_params, 0.08467)
    if len(tags) > 0:
        count_tag = count_tag + 1

    print(tags)
    count = count + 1
    if count > 19:
        print()
        print("20 times : ", time.time()-start_time)
        print("Tag seen : ", count_tag)
        break


    key = cv2.waitKey(1)
    if key == 27:
        break
 
cap.release()

