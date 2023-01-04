#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h" // gps velocity
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
// #include "dji_sdk/DroneArmControl.h"
// #include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
// #include "sensor_msgs/BatteryState.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
// #include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름
// #include "geometry_msgs/Vector3.h"
// #include "tf2_msgs/TFMessage.h"
// // multi thread
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <chrono>
// #include <string>

#include <sstream>
// 이미지 출력을 위한 임시 코드
#include "sensor_msgs/CameraInfo.h"
#include <time.h>
#include <math.h>
// 시뮬레이션 위치 파악용
#include "dji_sdk/VOPosition.h"

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

EulerAngles ToEulerAngles(Quaternion q);
// 쿼터니언 여기까지

// 수정 사항 및 코드 특징
// talker_sim_platform_position.cpp
// 플랫폼의 위치를 시뮬레이팅해서 Publish하는 코드.

// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

// 전역 변수 정의
// 처음에 /dji_sdk/sdk.launch 실행 되었는지 확인용
float imu_y = 0;
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;
float acc_x, acc_y, acc_z; // 오른쪽, 전진, 상 - 맞는지 확인(ENU는 아닌지)!!

using namespace std;

// VOPosition
float vo_x, vo_y, vo_z;
// Target position(simulation)
float target_x, target_y, target_z; // 동, 북
// starting position
float start_x, start_y, start_z; // E: x, N : y

// 제어 관련
double dt = 0;
double time_prev = 0;

// 플랫폼 경로
int mode = 1;
/*
mode = 0 : 5m 에 계속 위치
mode = 1 : 4m/s로 동쪽으로 등속 이동
mode = 2 : 반지름이 5m 인 원을 그린다. 시작은 원점
mode = 3 : 동쪽으로 3sin 의 경로를 그린다.
*/

// 시간 관련
float time_start;
float dt_all;
bool is_init = true;
int count_start = 0;
float time_constant = 0;

// 드론 속도
float vel_f, vel_r, vel_f_prev, vel_r_prev;
// 플랫폼 위치 및 속도
float dist_r, dist_f, dist_r_prev, dist_f_prev;
float vel_platform_r, vel_platform_f;
float vel_platform_r_abs, vel_platform_f_abs;
// 숫자 입력 받을때 까지 print 못하게 하기 위함
float is_number_ready = false;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// gps 관련
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);
void cb_voposition(const dji_sdk::VOPosition pos); // 위치
void cb_acc_ground(const geometry_msgs::Vector3Stamped acc);
float get_yaw(float w, float x, float y, float z);

void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void get_time();

// 방향 변환
float fru_to_enu(float f, float r, float theta);
float enu_to_fru(float e, float n, float theta);

// 위치 publish
std_msgs::Float64MultiArray target_pub;

int main(int argc, char **argv){
  cout << "Start platform simulation" << endl;

  ros::init(argc, argv, "talker_sim_platform_position");
  ros::NodeHandle nh;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = nh.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_vo_pos = nh.subscribe("dji_sdk/vo_position", 1000, cb_voposition);
  
  // publisher
  ros::Publisher platform_sim = nh.advertise<std_msgs::Float64MultiArray>("platform_sim", 10);

  // Publish rate?
  ros::Rate loop_rate(50);  

  // 무한루프 시작하기 전에 시뮬레이션 실행 됐는지 확인
  while(imu_y == 0){
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    time_start = ros::Time::now().toSec();
  }
  start_x = vo_x;
  cout << "Start_x : " << start_x << endl;
  start_y = vo_y;
  start_z = vo_z;

  // float time_prev = ros::Time::now().toSec();
  float time_start = ros::Time::now().toSec();
  
  // cout << time_start << endl;

  // 플랫폼 위치 출력
  while(ros::ok()){
    // float dt = ros::Time::now().toSec() - time_prev;
    // if(is_init){
    //   time_start = ros::Time::now().toSec();
    //   cout << time_start << endl;
    //   count_start += 1;
    //   if (count_start > 100){
    //     is_init = false;
    //   }
    // }
    float wait_time = 0.3;
    float dt_all_tmp = ros::Time::now().toSec() - time_start;
    if (is_init){
      time_constant = dt_all_tmp;
      is_init = false;
    }
    dt_all = dt_all_tmp - time_constant;
    if (dt_all < wait_time){
      // 1초 동안 이동 안함
      continue;
    }
    dt_all -= wait_time;
    // cout << dt_all << endl;
    // cout << ros::Time::now().toSec() - time_start << endl;
    // time_prev = ros::Time::now().toSec();
    float tx = 0;
    float ty = 0;
    float tz = 3.5;

    if(mode == 0){
      // 5m에 가만히 있는다.
      target_x = start_x + 5;
      target_y = start_y;
    }
    else if(mode == 1){
      float platform_a = 0.5;
      float target_spd = 3.0;

      float change_time = target_spd / platform_a;

      // // 3m/s 동쪽으로 등속 이동
      if(dt_all < change_time){
        target_x = 1/2 * (platform_a) * dt_all * dt_all;
      }
      else{
        target_x = 1/2 * (platform_a) * change_time * change_time + target_spd * (dt_all - change_time);
      }
      // target_x = start_x + 3 * dt_all;
      // cout << dt_all << endl;
      target_y = start_y;
    }
    else if(mode == 2){
      // 반지름이 5m 인 원을 그린다. 시작은 원점, 4m/s
      float r = 5;
      float w = 4/r;
      target_x = start_x + r * cos(w * dt_all) - r;
      target_y = start_y + r * sin(w * dt_all);
    }
    else if (mode == 3){
      // 동쪽으로 3sin 의 경로를 그린다. - 이건 안해도 될 듯
      
    }


    float te = target_x - vo_x;
    // cout << target_x << " " << vo_x << " " << te << " " << target_x - start_x << endl;
    float tn = target_y - vo_y;
    float yaw = get_yaw(imu_ori_w, imu_ori_x, imu_ori_y, imu_ori_z);
    ty, tx = enu_to_fru(te, tn, yaw);
    ty = -1 * ty; // 태그는 반대로 된 정보를 받아서 이렇게 해야함.
    tz = -1 * vo_z;

    target_pub.data = {tx,ty,tz,target_x,target_y};

    platform_sim.publish(target_pub);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;


  // // // 쓰레드 4 - Simulation position
  // auto simulation_position = []()
	// {
  //   float how_long = 10; // 초기 값을 10으로 해야 바로 아래줄에서 11초로 설정한게 오류가 안생긴다.
  //   ros::Time move_time_start = ros::Time::now() - ros::Duration(11);
  //   float distance_to_right;
  //   float distance_to_front = 0; // 지금은 사용 안할거라서 0으로 설정.
  //   // float vel_east; - 메인함수 정의하기 직전에 있는데, 그거 없애고 이거 주석 풀기
  //   float vel_north = 0; // 지금은 사용 안할거라서 0으로 설정.
  //   ros::Time update_position = ros::Time::now();
  //   float count_move = 0;

  //   while(is_run_thread){
  //     if(ros::Time::now() - move_time_start > ros::Duration(how_long)){ // 10초 이상 시간이 경과 되었을때
  //       // // 현재 드론 State 출력 중지
  //       // is_number_ready = false;

  //       // // 플랫폼 위치 초기화
  //       // target_x = vo_x;
  //       // target_y = vo_y;

  //       // // 변수 입력 받기
  //       // cout << "Input Kp" << endl;
  //       // cin >> kp_r;
  //       // cout << "Input kd" << endl;
  //       // cin >> kd_r;
  //       // cout << "Input distance to right" << endl;
  //       // cin >> distance_to_right;
  //       // cout << "Input platform velocity(m/s)" << endl;
  //       // cin >> vel_east;
  //       // cout << "How long(sec)?" << endl;
  //       // cin >> how_long;
  //       // if(how_long < 0){
  //       //   how_long = 10;
  //       // }
  //       // if(how_long > 60){
  //       //   how_long = 60;
  //       // }
        
  //       // Print 해도 된다는 뜻
  //       is_number_ready = true;

  //       // 현재 Heading
  //       EulerAngles angles;
  //       Quaternion q;
  //       q.w = imu_ori_w;
  //       q.x = imu_ori_x;
  //       q.y = imu_ori_y;
  //       q.z = imu_ori_z;

  //       angles = ToEulerAngles(q);
  //       float theta = angles.yaw;

  //       // 목표 위치
  //       target_x = vo_x + distance_to_right * sin(theta) + distance_to_front * cos(theta);
  //       target_y = vo_y - distance_to_right * cos(theta) + distance_to_front * sin(theta);

  //       // 시간 초기화
  //       move_time_start = ros::Time::now();
  //       update_position = ros::Time::now();
  //       count_move = 0.0001;
  //     }

  //     // 플랫폼 위치를 속도에 맞춰서 업데이트
  //     if(ros::Time::now() - update_position > ros::Duration(count_move)){
  //       target_x = target_x + 0.0001 * vel_east;

  //       count_move = count_move + 0.0001;
  //     }

  //   }
	// };

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

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_x = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_y = pos.x;
  vo_z = pos.z;
}

void spd(float a[],float b, float c, float d, float e){ // 파이썬 처럼 배열 변경하기 위해 만든 함수.
  a[0] = b; // 
  a[1] = c; // 
  a[2] = d; // 
  a[3] = e; // 
}

float get_yaw(float w, float x, float y, float z){
  float yaw_x = 2 * (w * z + x * y);
  float yaw_y = 1 - 2 * (y * y + z * z);
  float yaw = atan2(yaw_x, yaw_y);

  return yaw;
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
} // 쿼터니언 여기까지 (각도는 라디안)

// 방향 변환
float fru_to_enu(float f, float r, float theta){
  // 데이터는 flu 이기 때문에, 이거 쓸 일은 별로 없어보인다.
  float e = r*sin(theta) + f*cos(theta);
  float n = -1*r*cos(theta) + f*sin(theta);

  return e, n;
}

float enu_to_fru(float e, float n, float theta){
  float f = e*cos(theta) + n*sin(theta);
  float r = e*sin(theta) - n*cos(theta);

  return f, r;
}