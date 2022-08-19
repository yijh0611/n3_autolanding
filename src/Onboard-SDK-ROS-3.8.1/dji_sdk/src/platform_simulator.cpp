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

// void pub_tf(const tf2_msgs::TFMessage msg3); // Tag 정보 - 거리 기반 PID
void get_time();
float get_yaw(float w, float x, float y, float z);
float enu_to_fru(float e, float n, float y);

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
  // ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  // ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  // ros::Publisher distance_pub = n.advertise<std_msgs::Float64MultiArray>("distance", 10);
  // ros::Publisher pub_tf = n.advertise<tf2_msgs::TFMessage>("tf", 10); // 필요한거 같은데, 일단 주석
 
  // GPS신호 대기
  float chk_start = 0;
  is_run_thread = true;
  // while(vo_position_chk == 0){
  //   chk_start += 1;
  //   if (chk_start > 300){
  //     ROS_ERROR("No GPS position for 30 Sec!");
  //     is_run_thread = false;
  //     return 0;
  //     break;
  //   }
  //   ros::Duration(0.1).sleep();
  //   ros::spinOnce();
  // }
  
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
    cout << "Tag simulation start" << endl;

    // // 드론 제어
    ros::NodeHandle n;
    ros::Publisher pub_tf = n.advertise<geometry_msgs::Transform>("tf_2", 10);
    // ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10); // 이거로 해야지 NEU가 아니라 FRU로 할 수 있다.
    // ros::Publisher distance_pub = n.advertise<std_msgs::Float64MultiArray>("distance", 10);
    ros::Rate loop_rate(50);

    // time_prev = ros::Time::now().toSec() - 2; // 시작할 때 속도 높게나오는거 방지하기 위함.
    // ros::Time time_no_tag = ros::Time::now();
    // // float tag_time = ros::Time::now().toSec();
    // struct timeval time_now{};
    // time_t msecs_tag_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
    int i = 0;
    geometry_msgs::Transform tag_po;
    cout << "chk point" << endl;
    

    while(is_run_thread){ // Start Tag simulation
      i+=1;
      
      tag_po.translation.x = i;
      pub_tf.publish(tag_po);

      if (i>100){
        i = 0;
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

float enu_to_fru(float e, float n, float y){
  float f = e * cos(y) + n * sin(y);
  float r = e * sin(y) - n * cos(y);

  return f,r;
}

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_e = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_n = pos.x;
  vo_z = pos.z;
}