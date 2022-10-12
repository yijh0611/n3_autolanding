// 코드 정보
// 코드 이름 : platform_simulator
// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h" // gps velocity
#include "dji_sdk/DroneArmControl.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
// #include "sensor_msgs/BatteryState.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
#include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름

// 태그 정보 보내기
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"

// multi thread
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <string>

#include <sstream>
// 이미지 출력을 위한 임시 코드
#include "sensor_msgs/CameraInfo.h"
#include <time.h>
#include<math.h>
// 시간 ms 단위로 표현하기 위함
#include <sys/time.h>
#include <ctime>
// 시뮬레이션 위치 파악용
#include "dji_sdk/VOPosition.h"

// 난수 생성
// https://modoocode.com/304
#include <iomanip>
// #include <iostream>
// #include <map>
#include <random>

// Quaternion 계산용
#include <iostream>
using namespace std;

#define _USE_MATH_DEFINES
#include <cmath>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};
// 쿼터니언 여기까지

// void tf_callback(const turtlesim::PoseConstPtr& msg){

// 전역 변수 정의
float imu_y = 0;
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;

float hz = 12;
float dt_hz = 1/hz;
// float delayed_tag_time = 0.3;
// int how_many = delayed_tag_time * hz + 1;
float tag_f_arr[4]; // 여기 4 대신에 how_many로 바꿔도 됨
float tag_r_arr[4];

bool is_start = false;
using namespace std;
// Ctrl+C일때 바로 종료하기 위함.
bool is_run_thread;
// 현재 시간
int year, month, day, hour, minute, sec;
// 드론 속력 파악
float drone_vel_e, drone_vel_n;
float vo_position_chk = 0;
// 드론 위치
float vo_e, vo_n, vo_z;
// 태그 위치
float x_tf, y_tf, z_tf;
double time_prev;
// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// gps 관련
void cb_voposition(const dji_sdk::VOPosition pos); // 위치

// float get_yaw(float w, float x, float y, float z){
//   float yaw_x = 2 * (w * z + x * y);
//   float yaw_y = 1 - 2 * (y * y + z * z);
//   float yaw = atan2(yaw_x, yaw_y);

//   return yaw;
// }

// float enu_to_fru(float e, float n, float y){
//   float f = e * cos(y) + n * sin(y);
//   float r = e * sin(y) - n * cos(y);

//   return f,r;
// }

// void pub_tf(const tf2_msgs::TFMessage msg3); // Tag 정보 - 거리 기반 PID
void get_time();
float get_yaw(float w, float x, float y, float z);
float enu_to_fru_f(float e, float n, float y);
float enu_to_fru_r(float e, float n, float y);

// tf2_msgs::TFMessage tag_po; // 태그 정보 보내기 위한 변수

int main(int argc, char **argv){
  is_run_thread = true;
  ros::init(argc, argv, "Platform_simulator");
  ROS_INFO("Start Platform simulator");

  ros::NodeHandle n;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_vo_pos = n.subscribe("dji_sdk/vo_position", 1000, cb_voposition);
  
  // publisher
  // ros::Publisher pub_tf = n.advertise<tf2_msgs::TFMessage>("tf", 10); // 필요한거 같은데, 일단 주석
 
  // GPS신호 대기
  float chk_start = 0;
  is_run_thread = true;
  while(vo_position_chk == 0){
    chk_start += 1;
    if (chk_start > 300){
      ROS_ERROR("No GPS position for 30 Sec!");
      is_run_thread = false;
      return 0;
      break;
    }
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  if(chk_start <= 300){
    ROS_INFO("Vo_position recieved");
    cout << "vo_e : " << vo_e << endl;
    cout << "vo_n : " << vo_n << endl;
    cout << "vo_z : " << vo_z << endl;
  }
  
  // for(int i=0; i<3; i++){
  //   // tag_po.transforms[0].transform.translation.push_back(0.0);
  //   // 이렇게 하는건지 아니면
  //   // msg3.transforms[0].transform.translation.x = 1;
  //   // 이렇게 하는건지 잘 모르겟다
  //   // tag_po.data.push_back(0.0);
  // }
  // tag_po.transforms[0].transform.translation.x = 1;
  // pub_tf.publish(tag_po);
  // ros::Rate loop_rate(50);

  // // // // 쓰레드 1 - Get sensor data
  auto ros_spin = []()
	{
    if(is_run_thread){
      cout << "Start ros spin (sim)" << endl;
      ros::spin(); // 이거 굳이 쓰레드 한개를 차지할 필요가 있나? 그냥 메인함수에 남겨둬도 될거 같은데.
    }
    is_run_thread = false;
	};

  // 쓰레드 2 - Control drone
  auto control_drone = []()
	{
    double time_is_start = ros::Time::now().toSec();
    int int_wait = 0;
    ROS_INFO("Simulation wait 12s");
    while (ros::Time::now().toSec() - time_is_start < 12){
      int_wait = 1 - int_wait;
    }
    cout << "Tag simulation start" << endl;

    // // 드론 제어
    ros::NodeHandle n;
    ros::Publisher pub_tf = n.advertise<geometry_msgs::Transform>("tf_2", 10);
    ros::Publisher pub_sim_data = n.advertise<std_msgs::Float64MultiArray>("sim_data", 10);
    ros::Rate loop_rate(50);

    geometry_msgs::Transform tag_po;
    std_msgs::Float64MultiArray sim_data;

    for(int i=0; i < 15; i++){ // 5까지 하면 arr[4]까지 쓸 수 있음.
      sim_data.data.push_back(0.0);
    }

    double time_start = ros::Time::now().toSec();
    double time_prev = ros::Time::now().toSec();

    int i = 0;
    // int j = 0;
    bool is_first = true;
    
    float vo_e_start = 0;
    float vo_n_start = 0;
    float vo_z_start = 0;

    float dt_all = 0;
    float dt = 0;
    // bool is_circle = true;
    int what_direction = 1; // 0: circle, 1: square, 2: backward and forward

    float tag_e = 0;
    float tag_n = 0;
    float tag_f = 0;
    float tag_r = 0;
    float yaw = 0;
    float tag_vel_e = 2;
    float tag_vel_n = 0;

    int count_while = 0;
    int count_tag = 5;

    ROS_INFO("Start simulation");

    for(int i_for = 0; i_for < 5; i += 1){
      tag_f_arr[i_for] = 0;
      tag_r_arr[i_for] = 0;
    }

    while(is_run_thread){ // Start Tag simulation
      if (is_first){
        if (vo_z != 0){
          vo_z_start = vo_z;
          vo_e_start = vo_e;
          vo_n_start = vo_n + 1;

          tag_e = vo_e;
          tag_n = vo_n;

          is_first = false;

          time_start = ros::Time::now().toSec();
          time_prev = ros::Time::now().toSec();
          ROS_INFO("init");
        }
      }

      count_while += 1;

      // dt 랜덤하게 정의 - 정규분포로 난수 생성하는건 없는 것 같다.
      // 정규분포를 가지고 있는 list를 만들어서 그 중에 랜덤하게 꺼내는걸 해야 하는 듯.
      // 평균이 0이고 표준편차가 1인 난수
      
      // 난수 생성
      random_device rd;
      mt19937 gen(rd());
      normal_distribution<double> dist(/* 평균 = */ 0, /* 표준 편차 = */ 1);
      
      double rand_num_f = dist(gen) * 0.01;
      // rand_num_f = rand_num_f * 0.05;
      double rand_num_r = dist(gen) * 0.01;

      if (vo_z != 0){ // 위치 정보가 들어올 때
       
        // Tag 원운동 할 때
        // if (is_circle){
        //   int radius = 15;
        //   int ang_vel = 0.5;
        //   tag_vel_e = radius * cos(ang_vel * dt_all);
        //   tag_vel_n = radius * sin(ang_vel * dt_all); 
        // }

        // // 시간이 너무 지나면, 정지
        // if(dt_all > 100){
        //   tag_vel_e = 0;
        //   tag_vel_n = 0;
        // }

        // Tag 위치
        if(dt_all < 100){
          if(what_direction == 0){ // 원
            float radius = 15;
            // float ang_vel = 0.133;
            float ang_vel = 0.266;
            tag_e = vo_e_start + radius * sin(ang_vel * dt_all);
            tag_n = vo_n_start + radius - radius * cos(ang_vel * dt_all);
          }
          else if(what_direction == 1){ // 네모
            float dist = 10;
            float vel = 2;
            float distance_moved = dt_all * vel;
            if(distance_moved < dist){
              tag_e = vo_e_start + vel * dt_all;
              tag_n = vo_n_start;
            }
            else if(distance_moved < dist * 2){
              tag_e = vo_e_start + dist;
              tag_n = vo_n_start + vel * dt_all - dist;
            }
            else if(distance_moved < dist * 3){
              tag_e = vo_e_start + dist - (vel * dt_all - dist * 2);
              tag_n = vo_n_start + dist;
            }
            else if(distance_moved < dist * 4){
              tag_e = vo_e_start;
              tag_n = vo_n_start + dist - (vel * dt_all - dist * 3);
            }
            else{
              tag_e = vo_e_start;
              tag_n = vo_n_start;
            }
          }
          else if(what_direction == 2){ // 앞뒤로
            float dist = 10;
            float vel = 2;
            float distance_moved = dt_all * vel;
            if(distance_moved < dist){
              tag_e = vo_e_start + vel * dt_all;
              tag_n = vo_n_start;
            }
            else if(distance_moved < dist * 2){
              tag_e = vo_e_start + dist - (vel * dt_all - dist);
              tag_n = vo_n_start;
            }
            else if(distance_moved < dist * 3){
              tag_e = vo_e_start + (vel * dt_all - dist * 2);
              tag_n = vo_n_start;
            }
            else if(distance_moved < dist * 4){
              tag_e = vo_e_start+ dist - (vel * dt_all - dist * 3);
              tag_n = vo_n_start;
            }
            else{
              tag_e = vo_e_start;
              tag_n = vo_n_start;
            }
          }
          else{
            tag_e += tag_vel_e * dt;
            tag_n += tag_vel_n * dt;
          }
        }

        float drone_to_tag_e = tag_e - vo_e;
        float drone_to_tag_n = tag_n - vo_n;

        yaw = get_yaw(imu_ori_w, imu_ori_x, imu_ori_y, imu_ori_z);
        tag_f = enu_to_fru_f(drone_to_tag_e, drone_to_tag_n, yaw);
        tag_r = enu_to_fru_r(drone_to_tag_e, drone_to_tag_n, yaw);

        // data publish
        sim_data.data[0] = tag_f;
        sim_data.data[1] = tag_r;
        sim_data.data[2] = tag_e;
        sim_data.data[3] = tag_n;
        // sim_data.data[4] = vo_e;
        // sim_data.data[5] = vo_n;

        tag_f = tag_f + rand_num_f;
        tag_r = tag_r + rand_num_r;
        
        int where_arr = count_tag % 4;
        
        tag_f_arr[where_arr] = tag_f;
        tag_r_arr[where_arr] = tag_r;

        int where_arr_tmp = (count_tag - 3) % 4;
        tag_f = tag_f_arr[where_arr_tmp];
        tag_r = tag_r_arr[where_arr_tmp];

        count_tag += 1;

        tag_po.translation.x = tag_r; // 일단 이렇게 하고, flu로 바꿔서 다시 하기.
        tag_po.translation.y = -1 * tag_f;
        tag_po.translation.z = -1 * (vo_z - vo_z_start); // vo_z는 높아지면 음수가 되는 듯.

        // Publish Tag info
        pub_tf.publish(tag_po);
        pub_sim_data.publish(sim_data);
        

        if(count_while % 12 == 0){ // 정보 확인용.
        //   // cout << "Tag distance from start : " << tag_e - vo_e_start << endl;
        //   // cout << "tag_e : " << tag_e - vo_e << endl;
        //   // cout << "drone to tag e : " << drone_to_tag_e << endl;
        //   // cout << "drone to tag n : " << drone_to_tag_n << endl;
        //   // cout << "yaw : " << yaw << endl;
        //   // cout << "sin : " << sin(yaw) << endl;
        //   // cout << "cos : " << cos(yaw) << endl;
          cout << "Tag_f : " << tag_f << endl;
          cout << "Tag_r : " << tag_r << endl;
          if(fabs(tag_f - rand_num_f) > 2){
            cout << endl;
            ROS_ERROR("Too far F");
          }
          if(fabs(tag_r - rand_num_r) > 2){
            cout << endl;
            ROS_ERROR("Too far R");
          }
        //   // cout << endl;

        //   // cout << "vo_e : " << vo_e << endl;
        //   // cout << "vo_n : " << vo_n << endl;
        //   // // cout << "tag_n : " << tag_n - vo_n << endl;
          // cout << "tag_abs_e : " << tag_e << endl;
          // cout << "tag_abs_n : " << tag_n << endl;
          // cout << "dt_all : " << dt_all << endl;
        //   // cout << "count : " << count_while << endl;
          cout << endl;
          cout << endl;
          
        //   count_while = 0;
        }

        while(ros::Time::now().toSec() - time_prev < dt_hz){ // 12Hz로 데이터 보내기
          // 아무것도 안함.
          i = 1 - i;
        }
        dt_all = ros::Time::now().toSec() - time_start;
        dt = ros::Time::now().toSec() - time_prev;
        // cout << "dt : " << dt << endl;
        time_prev = ros::Time::now().toSec();
      }
    }
	};

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();

  return 0;
}

void get_imu(const sensor_msgs::Imu msg)
{
  imu_y = msg.orientation.y;

  imu_ori_x = msg.orientation.x;
  imu_ori_y = msg.orientation.y;
  imu_ori_z = msg.orientation.z;
  imu_ori_w = msg.orientation.w;
}

// void pub_tf(const tf2_msgs::TFMessage msg3) // need to be edited
// { // 값 저장하는 코드
//   x_tf = msg3.transforms[0].transform.translation.x;
//   y_tf = msg3.transforms[0].transform.translation.y;
//   z_tf = msg3.transforms[0].transform.translation.z;
// }

float get_yaw(float w, float x, float y, float z){
  float yaw_x = 2 * (w * z + x * y);
  float yaw_y = 1 - 2 * (y * y + z * z);
  float yaw = atan2(yaw_x, yaw_y);

  return yaw;
}

float enu_to_fru_f(float e, float n, float y){
  float f = e * cos(y) + n * sin(y);
  return f;
}

float enu_to_fru_r(float e, float n, float y){
  float r = e * sin(y) - n * cos(y);
  return r;
}

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_e = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_n = pos.x;
  vo_z = pos.z;
  vo_position_chk = pos.y;
}