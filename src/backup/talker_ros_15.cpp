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

EulerAngles ToEulerAngles(Quaternion q);
// 쿼터니언 여기까지

// 수정 사항 및 코드 특징
// talker_ros_15
// 착륙 관련 코드는 없다.
// ros_13에서 수정. - ros_13 기능은 유지하면서 추가
// GPS 정보랑 융합해서 현재 위치 및 속력등 파악하는 코드.
// D제어 부분 현재에서 이전거 빼게 수정

// 아직 작업 중인 부분
// GPS정보를 얼마나 믿을건지. 칼만필터 이용할건지. Low pass filter 이용할건지 등을 결정해야함.
// 이거를 먼저 한 뒤 목표 위치 까지 이동하는 기능 추가.
// !! Tag 거리 출력

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
time_t get_gps_time;
// ros::Time get_gps_time = ros::Time::now(); // GPS 신호 들어왔는지 파악하기 위함.
// GPS - gps velocity
float vel_x, vel_y, vel_z;
// VOPosition
float vo_x, vo_y, vo_z;
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
float reset_array(float cont[]); // 함수 정의만 하고 안쓰는 듯.
void callback_tf(const tf2_msgs::TFMessage msg3);
void cb_battery(const sensor_msgs::BatteryState battery);
void get_time();

std_msgs::Float64MultiArray gim;

int main(int argc, char **argv){

  is_run_thread = true;
  ros::init(argc, argv, "talker_ros_15");
  ros::NodeHandle n;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_gps_hlth = n.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_vel = n.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);
  // ros::Subscriber sub_gps_vel = n.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);
  ros::Subscriber sub_acc_ground = n.subscribe("dji_sdk/acceleration_ground_fused", 1000,cb_acc_ground);
  ros::Subscriber sub_battery = n.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
  
  // publisher
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);

  int chk_start = 0;
  // cout << "input speed" << endl;
  // cin >> spd_global;
  // cout << "input speed is :" << spd_global << endl;

  while(imu_y == 0){ // N3로부터 신호 수신이 안된다는 뜻. -> 지금은 그냥 넘어가는데, 나중에는 sdk.launch 재실행하게 하는 코드 넣어야 할 듯.!!
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 600){
      ROS_ERROR("No IMU signal for 1 min!");
      break;
    }
    ros::spinOnce();
  }

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
  
  // 계수 입력 받기
  cout << "Kp" << endl;
  cin >> kp_r;
  cout << "Kd" << endl;
  cin >> kd_r;
  kd_r = 0;
  cout << "Maximum speed" << endl;
  cin >> max_move_spd;
  if(max_move_spd > 10){
    max_move_spd = 10;
  }
  else if(max_move_spd < 0){
    max_move_spd = 1;
  }
  cout << "Kp : " << kp_r << endl;
  cout << "Kd : " << kd_r << endl;
  cout << "Maximum speed : " << max_move_spd << endl;
  // 계수 입력 여기까지
  
  ROS_INFO("Wait 2s");
  ros::Duration(2).sleep();
  bool obtain_control_result = obtain_control(); //비행 권한 얻는 코드
  if(!obtain_control_result){
     is_run_thread = false; // 권한을 못얻으면 코드 중지하기 위함.
  }

  // 짐벌 초기화
  gim.data.push_back(50); // 이거 해야지 크기 2인 배열 만들 수 있는 듯
  gim.data.push_back(0);
  gim.data[0] = 77;
  gim.data[1] = 76;
  gimbal_control.publish(gim); // 짐벌 초기화

  ros::Rate loop_rate(50);

  // // // // 쓰레드 1 - Get sensor data
  auto ros_spin = []()
	{
    if(is_run_thread){
      cout << "start ros spin" << endl;
      cout << "start ros spin" << endl;
      ros::spin(); // 이거 굳이 쓰레드 한개를 차지할 필요가 있나? 그냥 메인함수에 남겨둬도 될거 같은데.
    }
    is_run_thread = false; // 이거 오류 나는지 확인 !!
	};

  // 쓰레드 2 - Control drone
  auto control_drone = []()
	{
    cout << "start drone control" << endl;
    chk_x = 0;
    chk_y = 0;

    float move_left = 0.0;
    float move_right = 0.0;
    float move_front = 0.0;
    float move_speed = 0.5;

    if(spd_global < 10 && spd_global > 0){
      move_speed = spd_global;
    }

    ros::NodeHandle n;
    ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10); // 이거로 해야지 NEU가 아니라 FRU로 할 수 있다.
    ros::Rate loop_rate(50);

    ros::Time start_time = ros::Time::now();
    cout << "start moving" << endl;
    // while(ros::Time::now() - start_time < ros::Duration(1)){
    //   float cont[] = {move_speed,0.0,0.0,0.0}; //전좌상회전
    //   move(cont, n, control_pub);
    //   loop_rate.sleep();
    // }
    // cout << "move end" << endl;
    ros::Time is_no_tag_prev = ros::Time::now(); 
    while(is_run_thread){
      
      if(chk_x == x_tf){ // Tag를 못 찾았을 때
        if(ros::Time::now() - is_no_tag_prev > ros::Duration(0.5)){
          ROS_ERROR("No Tag!");
          is_no_tag_prev = ros::Time::now();
        }
        if(ros::Time::now() - is_no_tag_prev > ros::Duration(0.1)){ // 1초 기다리는거로 부족하면 다른 조건 더 추가해보기.
          move_left = 0.0;
          move_front = 0.0;

          // 속도 0으로 만들기. -> 근데 이렇게 되면 결국 25Hz로 제어하게 되는건가? !!
          // 아니면 continue 가 있으니까 괜찮은가?
        }
        float cont[] = {move_front,move_left,0.0,0.0}; //전좌상회전
        move(cont, n, control_pub);
        loop_rate.sleep();
        continue; // 이거 때문에 문제 되는건가?
      }
      else{
        is_no_tag_prev = ros::Time::now();
      }

      // 현재시간
      struct timeval time_now{};
      gettimeofday(&time_now, nullptr);
      time_t time_current = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
      
      // GPS 신호 새로 들어온지 0.2초 이내
      float time_now_float = (float) time_current;
      float gps_time_float = (float) get_gps_time;
      if(time_now_float - gps_time_float < 200){ // ms라서 200이 0.2s 이다.
        if(vel_f_prev != vel_f && vel_r_prev != vel_r){
          // 드론 속도 계산
          EulerAngles angles;
          Quaternion q;
          q.w = imu_ori_w;
          q.x = imu_ori_x;
          q.y = imu_ori_y;
          q.z = imu_ori_z;

          angles = ToEulerAngles(q);
          float theta = angles.yaw;
          // float theta_deg = theta * 3.14159265358979 / 180;

          vel_f = vel_x*cos(theta) + vel_y*sin(theta);
          vel_r = vel_x*sin(theta) - vel_y*cos(theta);

          vel_f_prev = vel_f;
          vel_r_prev = vel_r;
        }
        else{ // 가속도 이용
          vel_f = vel_f + acc_y * 0.01;
          vel_r = vel_r + acc_x * 0.01;
        }
      }
      else{ // 가속도 이용
        vel_f = vel_f + acc_y * 0.01;
        vel_r = vel_r + acc_x * 0.01;

        float time_now_float = (float) time_current;
        float gps_time_float = (float) get_gps_time;
        if(time_now_float - gps_time_float > 1000){ // GPS신호 1초 이상 안들어올때
          ROS_ERROR("No GPS Signal!");
        }
      }
      
      // if(chk_x == x_tf && chk_y == y_tf){ // chk_x == x_tf && chk_y == y_tf
      //   // 태그 0.1초 이내로 못찾으면 원래 속도 유지
      //   // 이거로 부족하면 화면 중앙에서 놓칠때 등등 여러가지 조건 추가해보기.
      // }

      dt = ros::Time::now().toSec() - time_prev;

      if(chk_x != x_tf){ // 좌우 PID 이동 코드
        
        if(fabs(x_tf) > 0.05){
          float d_x_tf = x_tf - x_tf_prev; // 현재에서 이전을 빼는게 맞나?
          float vx_tf = d_x_tf / dt;

          // cout << "vx_tf : " << vx_tf << endl;
          // 아래 이동 코드에다가 드론의 현재 속도에서 PID를 하게끔 만들어야 함.
          move_right = kp_r * x_tf + kd_r * vx_tf + ki_r * 0; // x_tf가 양수일때는 오른쪽으로 이동함
          if(fabs(move_right) > max_move_spd){
            move_right = max_move_spd * move_right / fabs(move_right);
          }
          move_left = -1 * move_right; // 제어 입력은 왼쪽으로 주기 때문에 -1을 곱해야 한다.
          if(x_tf > 0){
            cout << "Move right : " << x_tf << "\nspd : " << move_left << endl;
          }
          else{
            cout << "Move left : " << x_tf << "\nspd : " << move_left << endl;
          }
        }
        else{ // 거리가 가까울때 -> 이거는 등속으로 이동하게 만들어야 되는데, 아직 제어 방법을 정하지
        // 않았으므로 일단은 속도를 0으로 고정 !!
        // 근데 너무 가까울때 어떻게 할지에 대한 고민을 해야 되는지는 잘 모르겠다.
          move_left = 0.0;
          move_front = 0.0;
        }

        chk_x = x_tf;
        x_tf_prev = x_tf;
      }

      // 1초에 한번씩 배터리 퍼센트 표시하기
      if(ros::Time::now() - start_time > ros::Duration(1)){
        cout << "Battery : " << (batt_volt/1000) << " V" << endl;
        start_time = ros::Time::now();
      }

      // if(chk_y != y_tf){ // 앞뒤 이동 코드
      //   if(y_tf > 0.05){
      //     move_front = -1 * move_speed;
      //     cout << "Move backward " << y_tf << endl;
      //   }
      //   else if(y_tf < -0.05){
      //     move_front = move_speed;
      //     cout << "Move forward " << y_tf << endl;
      //   }
      //   chk_y = y_tf;
      // }

      float cont[] = {move_front,move_left,0.0,0.0}; //전좌상회전
      time_prev = ros::Time::now().toSec();
      move(cont, n, control_pub);
      loop_rate.sleep();

      // move_left = 0.0; // 이건 앞에 있는 if 문 안에 넣기로 했다.
      // move_front = 0.0;
    }
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

    // gimbal_x = 76;
    // gimbal_y = 56; // 바닥 보기
    // gimbal_x = 87; // 오른쪽 보기
    gimbal_x = 77; // 앞보기 - 핀 한개만 있는거. - 이거는 뺐음.(76이 정면) - 77으로 해야 정면 보는 듯
    gimbal_y = 76; // 앞 보기(77)
    x_tf = 0;
    y_tf = 0;
    chk_x = 0;
    chk_y = 0;
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

  // // 쓰레드 4 - Simulation position
  // auto simulation_position = []()
	// {
  //   while(is_run_thread){
  //     float a = 0.1;
  //   }
	// };

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  std::thread t3 = std::thread(control_gimbal);
  // std::thread t4 = std::thread(simulation_position);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  t3.join();
  // t4.join();
  // // // //// 여기까지

  return 0;
}

float reset_array(float cont[]){
  for (int i = 0;i < 4; i++){
    cont[i] = 0.0;
  }
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
  get_gps_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
}

void cb_voposition(const dji_sdk::VOPosition pos){
  cout << pos.x << endl;
  vo_x = pos.x;
  vo_y = pos.y;
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