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
#include <math.h>
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

EulerAngles ToEulerAngles(Quaternion q);
// 쿼터니언 여기까지

// 수정 사항 및 코드 특징
// talker_sim_4
// 목표 Input을 빠르게 만들기 위한 PD 제어기 사용시 개선이 되는지 확인하는 용

// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

// 전역 변수 정의
// 처음에 /dji_sdk/sdk.launch 실행 되었는지 확인용
float imu_y = 0;
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;
float acc_x, acc_y, acc_z; // 오른쪽, 전진, 상 - 맞는지 확인(ENU는 아닌지)!!

bool is_start = false;
using namespace std;
// int angle_gimbal; // 이거도 왜 있는거지?
// 태그 위치
float x_tf, y_tf;
// float z_tf; // 이거는 안쓰는거 같은데
// 짐벌 제어
float gimbal_x, gimbal_y;
// 태그 보고 있는지 확인
float chk_x, chk_y;
// 병렬 무한루프 중지하기 위함.
bool is_run_thread;
// 현재 시간
int year, month, day, hour, minute, sec;
// 드론 등속 속도
float spd_global = 0;
//cb_battery;
float batt_volt; // voltage
float batt_curr; // current
float batt_cap; // capacity
float batt_per; // percentage
// GPS - GPS_health
float gps_health;
// ros::Time get_gps_time = ros::Time::now();
// struct timeval time_now{};
// gettimeofday(&time_now, nullptr);
time_t msecs_time; // = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

// GPS - gps velocity
float vel_x, vel_y, vel_z;
// VOPosition
float vo_x, vo_y, vo_z;
// Target position(simulation)
float target_x, target_y; // 동, 북

// 제어 관련
float x_tf_prev = 0; // chk_x 랑 같은거
float y_tf_prev = 0;
float z_tf_prev = 0;
double dt = 0;
double time_prev = 0;
float kp_r = 1;
float kd_r = 0.5;
float ki_r = 0; // 이거는 아직 0
float max_move_spd = 1;
// 드론 속도
float vel_f, vel_r, vel_f_prev, vel_r_prev;
// 플랫폼 위치 및 속도
float dist_r, dist_f, dist_r_prev, dist_f_prev;
float vel_platform_r, vel_platform_f;
float vel_platform_r_abs, vel_platform_f_abs;
// 숫자 입력 받을때 까지 print 못하게 하기 위함
float is_number_ready = false;

// 드론 제어 권한
ros::ServiceClient sdk_ctrl_authority_service;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// gps 관련
void cb_gps_health(const std_msgs::UInt8 gps_h);
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);
void cb_voposition(const dji_sdk::VOPosition pos); // 위치
void cb_acc_ground(const geometry_msgs::Vector3Stamped acc);

void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void move(float cont[], ros::NodeHandle n, ros::Publisher control_pub);
// float reset_array(float cont[]); // 함수 정의만 하고 안쓰는 듯.
void callback_tf(const tf2_msgs::TFMessage msg3);
void cb_battery(const sensor_msgs::BatteryState battery);
void get_time();

// 방향 변환
float fru_to_enu(float f, float r, float theta);
float enu_to_fru(float e, float n, float theta);

std_msgs::Float64MultiArray target;

// !! 임시
float vel_east;

int main(int argc, char **argv){
  // get_time();
  cout << "Start main thread" << endl;
  is_run_thread = true;
  ros::init(argc, argv, "talker_sim_2");
  ros::NodeHandle n;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_gps_hlth = n.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_vel = n.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);
  ros::Subscriber sub_acc_ground = n.subscribe("dji_sdk/acceleration_ground_fused", 1000,cb_acc_ground);
  ros::Subscriber sub_battery = n.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  ros::Subscriber sub_vo_pos = n.subscribe("dji_sdk/vo_position", 1000, cb_voposition);
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
  
  // publisher
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  // ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);

  int chk_start = 0;

  cout << "Check IMU signal" << endl;
  while(imu_y == 0){ // N3로부터 신호 수신이 안된다는 뜻. -> 지금은 그냥 넘어가는데, 나중에는 sdk.launch 재실행하게 하는 코드 넣어야 할 듯.!!
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 600){
      ROS_ERROR("No IMU signal for 1 min!");
      break;
    }
    ros::spinOnce();
  }

  cout << "Check gps health" << endl;
  chk_start = 0;
  while(gps_health == 0){ // GPS 수신 안되면 일단 넘어가는데, 이거도 수정해야 할 듯.!!
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 600){
      ROS_ERROR("No GPS singal for 2 min!");
      break;
    }
    ros::spinOnce();
  }

  sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    
  // ROS_INFO("Wait 2s");
  // ros::Duration(2).sleep();
  bool obtain_control_result = obtain_control(); //비행 권한 얻는 코드
  if(!obtain_control_result){ 
     is_run_thread = false; // 권한을 못얻으면 코드 중지하기 위함.
  }

  // ros::Rate loop_rate(50); - 짐벌에 신호 보내려고 있는거 같긴한데, 필요할 수도 있을거 같아서 남겨뒀다.

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

    // 이동 명령
    ros::NodeHandle n;
    ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10); // 이거로 해야지 NEU가 아니라 FRU로 할 수 있다.
    // ros::Publisher target_pub = n.advertise<std_msgs::Float64MultiArray>("platform_position",10);
    ros::Rate loop_rate(50);
    float loop_time = 1/50;

    ros::Time control_start_time = ros::Time::now();
    cout << "start moving" << endl;
    
    double prop_prev = 0;
    double derivative = 0;
    double time_prev = ros::Time::now().toSec();

    while(is_run_thread){
      // vel_x 가 오른쪽 방향으로 현재 속력
      // 오른쪽으로 등속 직선운동

      float target_spd = 1;
      float kp = 4.5; // 4.5
      float kd = 0.5; // 0.5
      float prop = target_spd - vel_x;
      double dt = ros::Time::now().toSec() - time_prev;
      derivative = (prop - prop_prev) / dt;
      if(prop_prev == 0){
        kd = 0;
      }
      time_prev = ros::Time::now().toSec();

      float move_right = target_spd + kp * prop + kd * derivative; 
      prop_prev = prop;
      float move_left  = -1 * move_right;

      if(move_left < -5){
        move_left = -5;
      }

      float cont[] = {0.0, move_left, 0.0, 0.0}; // 전좌상회전
      
      move(cont, n, control_pub);
      if(ros::Time::now() - control_start_time > ros::Duration(10)){ // 10초간 이동
        // is_run_thread = false;
        break;
      }
    }

    control_start_time = ros::Time::now();
    while(is_run_thread){
      // 오른쪽으로 등속 직선운동
      float cont[] = {0.0,0.0,0.0,0.0}; // 전좌상회전
      
      move(cont, n, control_pub);
      if(ros::Time::now() - control_start_time > ros::Duration(10)){ // 10초간 이동
        is_run_thread = false;
        break;
      }
    }
	};

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

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  // std::thread t4 = std::thread(simulation_position);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  // t4.join();
  // // // //// 여기까지

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

void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel){
    vel_x = vel.vector.x;
    vel_y = vel.vector.y;
    vel_z = vel.vector.z;
}

void cb_gps_health(const std_msgs::UInt8 gps_h){
  gps_health = gps_h.data;
  // get_gps_time = ros::Time::now();
  struct timeval time_now{};
  gettimeofday(&time_now, nullptr);
  msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
}

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_x = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_y = pos.x;
  vo_z = pos.z;
}

void cb_acc_ground(const geometry_msgs::Vector3Stamped acc){
  acc_x = acc.vector.x;
  acc_y = acc.vector.y;
  acc_z = acc.vector.z;
}

void spd(float a[],float b, float c, float d, float e){ // 파이썬 처럼 배열 변경하기 위해 만든 함수.
  a[0] = b; // 
  a[1] = c; // 
  a[2] = d; // 
  a[3] = e; // 
}

void callback_tf(const tf2_msgs::TFMessage msg3) // need to be edited
{ // 값 저장하는 코드
  x_tf = msg3.transforms[0].transform.translation.x;
  y_tf = msg3.transforms[0].transform.translation.y; //z_tf는 없어도 되는건가?
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
  
  ros::Duration(0.02).sleep();
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
  batt_curr = battery.current;
  batt_cap = battery.capacity;
  batt_per = battery.percentage;
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