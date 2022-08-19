#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h" // gps velocity
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "dji_sdk/DroneArmControl.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
#include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름
#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"
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

// EulerAngles ToEulerAngles(Quaternion q);
// 쿼터니언 여기까지

// 코드 정보
// 코드 이름 : talker_ros_17
// PID 계수 P:0.7, D:0.07
// 칼만필터에서 얻은 정보 이용해서 제어 + 고도는 3.5m로 유지
// move함수안에 0.02초 기다리는거 있는데, 빼도 괜찮은지 확인 !!
// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

// 전역 변수 정의
// 처음에 /dji_sdk/sdk.launch 실행 되었는지 확인용
float imu_y = 0;
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;

bool is_start = false;
using namespace std;
// 짐벌 제어
float gimbal_x, gimbal_y;
// Ctrl+C일때 바로 종료하기 위함.
bool is_run_thread;
// 현재 시간
int year, month, day, hour, minute, sec;
//cb_battery;
float batt_volt; // voltage
// GPS_Health
float gps_health;
time_t get_gps_time;
// Tag position
float r_prev = 0;
float f_prev = 0;
float z_prev = 0;
float dt = 0;
// PID 계수
float kp = 0.7; 
float kd = 0.07; // 2.5
float ki = 0; // 이거는 아직 0
float max_move_spd = 5;
// 칼만필터
float kal_r,kal_f,kal_r_vel,kal_f_vel,kal_time,kal_is_tag,kal_is_tag_lost;
// 태그 위치
float x_tf, y_tf, z_tf;
double time_prev;
float gimbal_down;
bool is_gimbal_down = false; // 밖에서 할때는 false

// 드론 제어 권한
ros::ServiceClient sdk_ctrl_authority_service;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// gps 관련
void cb_gps_health(const std_msgs::UInt8 gps_h);

void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void move(float cont[], ros::NodeHandle n, ros::Publisher control_pub);
void cb_battery(const sensor_msgs::BatteryState battery);
void cb_kalman(const std_msgs::Float64MultiArray kal);
void callback_tf(const tf2_msgs::TFMessage msg3); // Tag 정보 - 거리 기반 PID
void get_time();

std_msgs::Float64MultiArray gim;
std_msgs::Float64MultiArray distance_data;

int main(int argc, char **argv){
  if(is_gimbal_down){
    gimbal_down = 76;
  }
  else{
    gimbal_down = 56;
  }
  
  is_run_thread = true;
  ros::init(argc, argv, "talker_ros_15");
  ros::NodeHandle n;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_gps_hlth = n.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_battery = n.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  ros::Subscriber sub_kalman = n.subscribe("kalmanfilter", 1000, cb_kalman);
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
  
  // publisher
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  ros::Publisher distance_pub = n.advertise<std_msgs::Float64MultiArray>("distance", 10);
 
  // 안전 검사
  int chk_start = 0;
  while(imu_y == 0){ // N3로부터 신호 수신이 안된다는 뜻. -> 지금은 그냥 넘어가는데, 나중에는 sdk.launch 재실행하게 하는 코드 넣어야 할 듯.!!
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 300){
      ROS_ERROR("No IMU signal for 30 sec!");
      is_run_thread = false;
      break;
    }
    ros::spinOnce();
  }


  chk_start = 0;
  while(gps_health == 0){ // GPS 수신 안되면 일단 넘어가는데, 이거도 수정해야 할 듯.!!
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 300){
      ROS_ERROR("No GPS singal for 30 Sec!");
      is_run_thread = false;
      break;
    }
    ros::spinOnce();
  }
  
  if(!is_run_thread){
    return 0;
  }
  // 안전검사 여기까지
  
  // 짐벌 초기화
  gim.data.push_back(50); // 이거 해야지 크기 2인 배열 만들 수 있는 듯
  gim.data.push_back(0);
  // gim.data[0] = 56; // 아래 바라보기 - 76이면 정면
  // gim.data[0] = 76; // 아래 바라보기 - 76이면 정면 - 나가서 할때는 주석
  gim.data[0] = gimbal_down;
  gim.data[1] = 77; // 좌우 77이면 정면
  gimbal_control.publish(gim); // 짐벌 초기화

  for(int i=0; i<5; i++){
    distance_data.data.push_back(0.0);
  }
  

  // // 계수 입력 받기
  cout << "Maximum speed" << endl;
  cin >> max_move_spd;

  if(max_move_spd > 5){
    max_move_spd = 5;
  }
  else if(max_move_spd < 0){
    max_move_spd = 1;
  }
  
  cout << "Maximum speed : " << max_move_spd << endl;
  // 계수 입력 여기까지

  // 드론 제어 권한
  sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    
  bool obtain_control_result = obtain_control(); //비행 권한 얻는 코드
  if(!obtain_control_result){
     is_run_thread = false; // 권한을 못얻으면 코드 중지하기 위함.
  }


  ros::Rate loop_rate(50);

  // // // // 쓰레드 1 - Get sensor data
  auto ros_spin = []()
	{
    if(is_run_thread){
      cout << "start ros spin" << endl;
      ros::spin(); // 이거 굳이 쓰레드 한개를 차지할 필요가 있나? 그냥 메인함수에 남겨둬도 될거 같은데.
    }
    is_run_thread = false; // 이거 오류 나는지 확인 !!
	};

  // 쓰레드 2 - Control drone
  auto control_drone = []()
	{
    cout << "start drone control" << endl;

    // Kalmanfilter 신호가 최신인지 파악하기 위함.
    float kal_time_tmp = kal_time - 1; // 이거는 4자리? + 소수점 아래 2자리 이기 때문에, 오버플로우 현상이 있을 수 있다.
                                   // 관련 문제 해결하는 방법 생각해보기.

    float move_left = 0.0;
    float move_right = 0.0;
    float move_front = 0.0;

    float time_limit = 0.5; // 0.5초 이상 태그가 안보이면 정지

    // 드론 제어
    ros::NodeHandle n;
    ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10); // 이거로 해야지 NEU가 아니라 FRU로 할 수 있다.
    ros::Publisher distance_pub = n.advertise<std_msgs::Float64MultiArray>("distance", 10);
    ros::Rate loop_rate(50);

    ros::Time control_time = ros::Time::now();
    ros::Time battery_time = ros::Time::now();
    cout << "start moving" << endl;

    float tag_x_tmp;
    float d_x_prev, d_y_prev;

    time_prev = ros::Time::now().toSec() - 2; // 시작할 때 속도 높게나오는거 방지하기 위함.
    ros::Time time_no_tag = ros::Time::now();
    while(is_run_thread){ // 이게 수정된 제어 코드

      if(ros::Time::now() - time_no_tag > ros::Duration(time_limit)){
        float cont[] = {0.0,0.0,0.0,0.0}; //전좌상회전
        move(cont, n, control_pub);
      }

      // 칼만필터 없이 태그만 가지고 이동
      if(tag_x_tmp != x_tf){ // 태그 정보가 바뀌었을때
        float d_x = x_tf;
        float d_y = -1 * y_tf;
        
        dt = ros::Time::now().toSec() - time_prev;
        float d_dx = d_x - d_x_prev;
        float d_dy = d_y - d_y_prev;
        d_x_prev = d_x;
        d_y_prev = d_y;
        time_prev = ros::Time::now().toSec();
        
        float v_x = d_dx / dt;
        float v_y = d_dy / dt;

        if(dt > 0.5){
          v_x = 0;
          v_y = 0;
        }

        move_right = kp * d_x + kd * v_x;
        move_front = kp * d_y + kd * v_y;

        distance_data.data[0] = d_x;
        distance_data.data[1] = v_x;
        distance_data.data[2] = d_y;
        distance_data.data[3] = v_y;
        distance_data.data[4] = dt;
        distance_pub.publish(distance_data);

        // 여기까지 거리 기반

        // 칼만필터
        if(kal_is_tag_lost == 0){ // 태그 놓치지 않았을 때
          float spd_constant = 0.7;

          kal_r_vel = kal_r_vel * spd_constant;
          kal_f_vel = kal_f_vel * spd_constant;
          
          move_right = kal_r_vel + move_right;
          move_front = kal_f_vel + move_front;
        }
        else{
          ROS_ERROR("Tag is lost. No more speed control");
        }

        // 드론 최대 속도
        if(fabs(move_right) > max_move_spd){
          move_right = max_move_spd * move_right / fabs(move_right);
        }

        if(fabs(move_front) > max_move_spd){
          move_front = max_move_spd * move_front / fabs(move_front);
        }

        float move_up = 0.0;
        
        // 태그보고 고도 조절
        if(fabs(z_tf - 3.5) > 0.1){
          move_up = (3.5 - z_tf) * 0.3;
        }

        move_left = -1 * move_right;

        float cont[] = {move_front,move_left,move_up,0.0}; //전좌상회전
        move(cont, n, control_pub);
        
        tag_x_tmp = x_tf;
        time_no_tag = ros::Time::now();
      }
      
      // 1초에 한번씩 배터리 퍼센트 표시하기
      if(ros::Time::now() - battery_time > ros::Duration(1)){
        cout << "Battery : " << (batt_volt/1000) << " V" << endl;
        battery_time = ros::Time::now();
        if(batt_volt/1000 < 14.5){
          float cont[] = {0.0,0.0,0.0,0.0}; //전좌상회전
          move(cont, n, control_pub); // 정지
          break;
        }
      }
    } // while 문
	};

  // 쓰레드 3 - Control gimbal
  auto control_gimbal = []()
	{
    /*
    상하로 제어할 때는 56~77까지이다.
    56일때 아래, 77일때 정면

    짐벌 좌우로 제어 가능한 범위는 56~96이다.
    56일때 드론 기준 왼쪽, 96일때 드론 기준 오른쪽
    */

    cout << "start gimbal control" << endl;
		
    ros::NodeHandle n;
    ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);

    // gimbal_x = 87; // 오른쪽 보기
    // gimbal_y = 56; // 바닥 보기
    gimbal_x = 77; // 앞보기 - 핀 한개만 있는거. - 이거는 뺐음.(76이 정면) - 77으로 해야 정면 보는 듯
    // gimbal_y = 56; // 앞 보기(77)76
    // gimbal_y = 76; // 앞 보기(77)76
    gimbal_y = gimbal_down;

    gim.data[0] = gimbal_y;
    gim.data[1] = gimbal_x;
    // cout << "look ground" << endl;
    gimbal_control.publish(gim);
    ros::Duration(0.5).sleep();

    while(is_run_thread){ // 원래는 여기에 짐벌 제어하는 코드가 있음.
      gim.data[1] = gimbal_x;
      gim.data[0] = gimbal_y;

      gimbal_control.publish(gim); // 값이 바뀌는것과 상관 없이 같은 값 publish

      ros::Duration(0.5).sleep(); // sleep for hundredth of a second
    }
	};

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  std::thread t3 = std::thread(control_gimbal);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  t3.join();

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

void cb_gps_health(const std_msgs::UInt8 gps_h){
  gps_health = gps_h.data;
  // get_gps_time = ros::Time::now();
  struct timeval time_now{};
  gettimeofday(&time_now, nullptr);
  get_gps_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
}

void spd(float a[],float b, float c, float d, float e){ // 파이썬 처럼 배열 변경하기 위해 만든 함수.
  a[0] = b; // 
  a[1] = c; // 
  a[2] = d; // 
  a[3] = e; // 
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("Obtain control failed!");
    return false;
  }
  else{
    ROS_INFO("Obtain control success!");
  }
  
  return true;
}

void move(float control[],ros::NodeHandle n,ros::Publisher control_pub){
  ros::Rate loop_rate(50);
  sensor_msgs::Joy msg_control;
  
  uint8_t flag = (DJISDK::VERTICAL_VELOCITY   | // when using generic
                  DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_RATE            |
                  DJISDK::HORIZONTAL_BODY  |
                  DJISDK::STABLE_ENABLE);
  
  msg_control.axes.push_back(control[0]);
  msg_control.axes.push_back(control[1]);
  msg_control.axes.push_back(control[2]);
  msg_control.axes.push_back(control[3]);
  msg_control.axes.push_back(flag);
  
  ros::Duration(0.02).sleep(); // 이거 왜 있는거지? !!
  control_pub.publish(msg_control);
  
  ros::spinOnce();
}

void get_time(){
  time_t rawTime;
  struct tm* pTimeInfo;

  rawTime = time(NULL);
  pTimeInfo = localtime(&rawTime);

  year = pTimeInfo->tm_year + 1900;    //연도에는 1900 더해줌
  month = pTimeInfo->tm_mon + 1;    // 월에는 1 더해줌
  day = pTimeInfo->tm_mday;
  hour = pTimeInfo->tm_hour;
  minute = pTimeInfo->tm_min;
  sec = pTimeInfo->tm_sec;
}

void cb_battery(const sensor_msgs::BatteryState battery){
  batt_volt = battery.voltage;
}

void cb_kalman(const std_msgs::Float64MultiArray kal){
  kal_f = kal.data[0];
  kal_r = kal.data[1];
  kal_f_vel = kal.data[2];
  kal_r_vel = kal.data[3];
  kal_time = kal.data[4];
  kal_is_tag = kal.data[5];
  kal_is_tag_lost = kal.data[6];
}

void callback_tf(const tf2_msgs::TFMessage msg3) // need to be edited
{ // 값 저장하는 코드
  x_tf = msg3.transforms[0].transform.translation.x;
  y_tf = msg3.transforms[0].transform.translation.y;
  z_tf = msg3.transforms[0].transform.translation.z;
}

