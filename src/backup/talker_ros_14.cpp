#include "ros/ros.h"
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

// 수정 사항 및 코드 특징
// talker_ros_14
// 카메라 아래로 고정하는거만 할 예정

// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

// 전역 변수 정의
// 처음에 /dji_sdk/sdk.launch 실행 되었는지 확인용
float imu_y = 0; 
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

// 드론 제어 권한
ros::ServiceClient sdk_ctrl_authority_service;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void move(float cont[], ros::NodeHandle n, ros::Publisher control_pub);
float reset_array(float cont[]); // 함수 정의만 하고 안쓰는 듯.
void callback_tf(const tf2_msgs::TFMessage msg3);
void cb_battery(const sensor_msgs::BatteryState battery);
void get_time();

std_msgs::Float64MultiArray gim;

int main(int argc, char **argv){

  is_run_thread = true;
  ros::init(argc, argv, "talker_ros_14");
  ros::NodeHandle n;

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_battery = n.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  
  int chk_start = 0;

  while(imu_y == 0){
    ros::Duration(0.1).sleep();
    chk_start += 1;
    if (chk_start > 120){
      break;
    }
    ros::spinOnce();
  }

  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);

  // publisher
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);

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
      ros::spin(); // 이거 굳이 쓰레드 한개를 차지할 필요가 있나? 그냥 메인함수에 남겨둬도 될거 같은데.
    }
    is_run_thread = false; // 이거 오류 나는지 확인 !!
	};

  // 쓰레드 2 - Control drone
  auto control_drone = []()
	{
    cout << endl;
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
    gimbal_y = 56; // 앞 보기(77)
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

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  std::thread t3 = std::thread(control_gimbal);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  t3.join();
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
