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

# imu
from sensor_msgs.msg import Imu
# 가속도, GPS
from geometry_msgs.msg import Vector3Stamped
# # AprilTag
from tf2_msgs.msg import TFMessage
# # Velocity Publish
from std_msgs.msg import Float64MultiArray
# vo position
from dji_sdk.msg import VOPosition
# # Test
# from std_msgs.msg import String


# IMU에서 Orientation 받아오기
imu_orientation = 0 # 처음부터 IMU센서 값으로 정의 어떻게 하지?
def callback_imu(imu_data):
    global imu_orientation
    imu_orientation = imu_data.orientation # x,y,z,w

acc = 0
def callback_acc(acc_data):
    global acc
    acc = acc_data.vector # x,y,z
    # print(acc)

gps_vel = 0
def callback_gps(gps_data):
    global gps_vel
    gps_vel = gps_data.vector # x,y,z

tf_count_time = time.time()
tf_count = 0
tf_tmp = 0
tf = 0
def callback_tf(tf_data):
    global tf
    tf = tf_data.transforms[0].transform.translation # x,y,z
    global tf_tmp
    global tf_count
    global tf_count_time
    if tf_tmp != tf:
        tf_tmp = tf
        tf_count += 1
    if time.time() > tf_count_time + 1:
        # print("Tags per sec : ",tf_count)
        tf_count = 0
        tf_count_time = time.time() 
    # # print("tf signal", time.time())

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

vo_po = 0
def callback_vo_po(vo_data):
    global vo_po
    vo_po = vo_data # x,y,z

# # 쿼터니언을 오일러로 바꾸는 함수
def Quaternion2EulerAngles(w, x, y, z):
    PIby2 = (math.pi / 2)

    # roll : rotation in x axis
    roll_x = float(2 * (w * x + y * z))
    roll_y = float(1 - 2 * (x * x + y * y))
    roll = math.atan2(roll_x, roll_y)

    # pitch : rotation in y axis
    pitch_var = float(2 * (w * y - z * x))
    if (abs(pitch_var) >= 1):
	#In the event of out of range -> use 90 degrees
        pitch = math.copysign(PIby2, pitch_var)
    else:
        pitch = math.asin(pitch_var)

    # yaw : rotation in z-axis
    yaw_x = 2 * (w * z + x * y)
    yaw_y = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(yaw_x, yaw_y)

    return roll, pitch, yaw

# 방향 변환
def fru_to_enu(f, r, theta):
    e = r*math.sin(theta) + f*math.cos(theta)
    n = -1*r*math.cos(theta) + f*math.sin(theta)
    
    return e, n
def enu_to_fru(e, n, theta):
  f = e*math.cos(theta) + n*math.sin(theta)
  r = e*math.sin(theta) - n*math.cos(theta)

  return f, r

def ros_spin_thread():
    print("Ros spin thread start")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)
    print('Start Kalman filter node')

    # IMU, 가속도(수정), AprilTag, GPS속력 등 속력 예측에 필요한 정보 받아오기
    rospy.Subscriber("dji_sdk/imu", Imu, callback_imu)
    rospy.Subscriber("dji_sdk/acceleration_ground_fused", Vector3Stamped, callback_acc)
    rospy.Subscriber("dji_sdk/velocity", Vector3Stamped, callback_gps)
    rospy.Subscriber("dji_sdk/vo_position", VOPosition, callback_vo_po)
    rospy.Subscriber("tf", TFMessage, callback_tf)

    # Publish
    pub_kalman = rospy.Publisher('kalmanfilter', Float64MultiArray, queue_size=10)
    # pub = rospy.Publisher('chatter', String, queue_size=10)

    # 현재시간, Platform의 x,y 속력 정보를 한번에 보내주기
    # print('Start Kalmanfilter node') # -> 멀티 쓰레드 만든 뒤에 하기.
    print('Check point')
    
    # 멀티쓰레드 만들어서 센서 값 받은거 계산해서 출력하게 만들기
    t = threading.Thread(target=ros_spin_thread)
    t.daemon = True
    t.start()
    
    # 여기다가 센서 값 통합해서 계산해가지고 보내주는거 만들기
    # 만약에 센서 값들이 정수라면 센서 값 못 받는다는 뜻.
    
    # IMU 받았는지 확인
    is_imu = True
    time_wait_sensor = time.time()
    while imu_orientation == 0:
        if time.time() - time_wait_sensor > 20:
            is_imu = False
            print("No IMU")
            break

    time.sleep(0.5) # 모든 센서값이 다 표시 되는지 확인할 수는 없어서 0.5초 대기 후 실행

    is_gps_acc = True
    time_wait_sensor = time.time()
    while gps_vel == 0:
        if time.time() - time_wait_sensor > 20:
            is_gps_acc = False
            print("No GPS")
            break
    
    if is_imu == False:
        is_gps_acc = False

    if is_gps_acc:
        # Tag 정보 받았는지 확인
        is_tag = False # False이면 태그가 한번도 보이지 않았다는 뜻.
        is_tag_lost = False # Ture 이면 한번 보인적이 있는데, 0.5초 이상 보이지 않았다는 뜻
                            # 드론이 10m/s으로 이동을 한다고 하면, 0.5초동안 보이지 않은 경우
                            # 5m 정도를 이동한다.
                            # True가 되면, 태그를 놓쳤다는 메세지 Publish하기.
        
        roll, pitch, yaw = Quaternion2EulerAngles(imu_orientation.w,imu_orientation.x,imu_orientation.y,imu_orientation.z)
        acc_e, acc_n = fru_to_enu(acc.y, acc.x, yaw)

        # 칼만필터 정의
        dt = 0.01
        my_filter = KalmanFilter(dim_x=10, dim_z=8)
        my_filter.x = np.array([[vo_po.x], [vo_po.y], [gps_vel.x], [gps_vel.y], [acc_e], [acc_n], [0.0], [0.0], [0.0], [0.0]]) # initial state (location and velocity)
                            # [[right[0]], [front[0]], [vel_r[0]], [vel_f[0]], [acc_r[0]], [acc_f[0]], [tag_r[0]], [tag_f[0]], [0.0], [0.0]]
        my_filter.F = np.array([[1., 0., dt, 0., dt*dt/2,      0., 0., 0., 0., 0.],
                                [0., 1., 0., dt,      0., dt*dt/2, 0., 0., 0., 0.],
                                [0., 0., 1., 0.,      dt,      0., 0., 0., 0., 0.],
                                [0., 0., 0., 1.,      0.,      dt, 0., 0., 0., 0.],
                                [0., 0., 0., 0.,      1.,      0., 0., 0., 0., 0.],
                                [0., 0., 0., 0.,      0.,      1., 0., 0., 0., 0.],
                                [0., 0., 0., 0.,      0.,      0., 1., 0., dt, 0.],
                                [0., 0., 0., 0.,      0.,      0., 0., 1., 0., dt],
                                [0., 0., 0., 0.,      0.,      0., 0., 0., 1., 0.],
                                [0., 0., 0., 0.,      0.,      0., 0., 0., 0., 1.]])    # state transition matrix
        my_filter.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0.]])
        my_filter.P *= 100 * np.identity(10)                 # covariance matrix # 1000 * 단위행렬 ## 20* 단위행렬
        my_filter.R = 10 * np.identity(8)               # state uncertainty - # 0.001 # 10000
        my_filter.Q = 0.1 * np.identity(10)   # process uncertainty // Q = [1 0; 0 1000]; - 작게 해보기, 크면 측정값에 의존 # 0.00001
    
    # Tag 정보 받았는지 확인
    imu_chk = 0
    tag_chk = 0
    reset_tag_x = 0
    reset_tag_y = 0
    time_tag = time.time()
    tag_tmp = 0
    kal_dt = 0
    tag_vx = 0
    tag_vy = 0
    tag_vz = 0

    is_tag_lost = True

    # 무한루프 실행
    while is_gps_acc:
        if is_tag: # 태그가 보인적이 있는 경우
            if tag_chk == tf.x: # 태그가 보이지 않는 경우
                if time.time() - time_tag > 0.5: # 0.5초 이상 Tag 값 일정
                    is_tag_lost = True
                    # print("Tag is lost", time.time()-time_tag)
            else: # 태그가 보이는 경우
                is_tag_lost = False
                tag_chk = tf.x
                kal_dt = time.time() - time_tag
                if tag_tmp != 0:
                    tag_vx = (tf.x - tag_tmp.x)/kal_dt
                    tag_vy = (tf.y - tag_tmp.y)/kal_dt
                    tag_vz = (tf.z - tag_tmp.z)/kal_dt
                time_tag = time.time()
                tag_tmp = tf
        else:# 태그가 한번도 보이지 않음.
            if tf != 0: # Tag를 확인했음
                is_tag = True
                tag_chk = tf.x
                reset_tag_x = tf.x
                reset_tag_y = tf.y
                tf.x = 0
                time_tag = time.time()
                print("Tag is found")

        # IMU 기반 Orientation 계산
        roll, pitch, yaw = Quaternion2EulerAngles(imu_orientation.w,imu_orientation.x,imu_orientation.y,imu_orientation.z)

        # Orientation 기반 드론 방향 속도 계산
        # 가속도
        acc_e, acc_n = fru_to_enu(acc.y, acc.x, yaw)
        
        # Tag
        if is_tag:
            tag_r = tf.x - reset_tag_x
            tag_f = -1 * (tf.y - reset_tag_y)
        else:
            tag_r = 0
            tag_f = 0
        
        tag_e, tag_n = fru_to_enu(tag_f, tag_r, yaw)
        
        # 칼만필터 업데이트
        my_filter.predict()
        x = np.array([[vo_po.x], [vo_po.y], [gps_vel.x], [gps_vel.y], [acc_e], [acc_n], [tag_e], [tag_n]]) # initial state (location and velocity)
        my_filter.update(x)
        
        x_filtered = my_filter.x

        # 플랫폼 속도 출력
        tag_f, tag_r = enu_to_fru(x_filtered[6],x_filtered[7],yaw)
        vel_f, vel_r = enu_to_fru(x_filtered[8]+x_filtered[2],x_filtered[9]+x_filtered[3],yaw)

        # C에서 float은 소수점 아래 6자리만 표현 가능?
        time_publish = time.time() % 1000 # 이렇게 해도 소수점 아래로 다 안나오면 1000으로 바꾸기
                                            # 아마 필요 없을 듯
        time_publish = round(time_publish,2)


        data_pub = Float64MultiArray()
        try:
            data_pub.data = np.array([tag_f - reset_tag_y, tag_r + reset_tag_x, vel_f, vel_r, time_publish, is_tag, is_tag_lost,tf.x,tf.y,tf.z,kal_dt,tag_vx,tag_vy,tag_vz])
        except:
            data_pub.data = np.array([tag_f - reset_tag_y, tag_r + reset_tag_x, vel_f, vel_r, time_publish, is_tag, is_tag_lost,0.0,0.0,0.0,kal_dt,tag_vx,tag_vy,tag_vz])
        pub_kalman.publish(data_pub)


