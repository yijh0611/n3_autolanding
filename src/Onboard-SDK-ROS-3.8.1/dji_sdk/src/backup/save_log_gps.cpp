#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
// #include "dji_sdk/DroneArmControl.h"
// 이미지
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
// #include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "geometry_msgs/QuaternionStamped.h"
// #include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
// #include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름
#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"
// 드론 이동 명령 확인
#include "sensor_msgs/Joy.h"

// CSV 저장
#include <fstream>
// // multi thread
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <string>

#include <sstream>
#include <time.h>

// 수정 사항 및 코드 특징
// save_log_gps - gps 관련 정보만 저장하는 코드

// 멀티쓰레드로 로그 저장
// 영상 받고 시간 적은 뒤 저장
// 센서 정보 받아오기

// 아직 작업 중인 부분
// GPS 관련 정보 저장하게 만들기.
// 파일 이름 구분하기

// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

using namespace std;
// 전역 변수 정의
bool is_test = true;
bool is_ctrl_c;
// 태그 위치
float x_tf, y_tf, z_tf;
float tf_rot_x, tf_rot_y, tf_rot_z, tf_rot_w;
// 현재 시간
int year, month, day, hour, minute, sec;
double m_sec;
// GPS - GPS_health
float gps_health;
// GPS - local position
float loc_po_x, loc_po_y, loc_po_z;
// GPS - gps position
float gps_po_stat, gps_po_lat, gps_po_lon, gps_po_alt;
// GPS - gps velocity
float vel_x, vel_y, vel_z;
// IMU
float imu_ori_y = 0;

// 로그 이름에 필요해서 지우면 안됨
string src_vid, src_log;

// 함수 정의
bool get_time();
bool get_src();
void cb_gps_health(const std_msgs::UInt8 gps_h);
void cb_gps_local_position(const geometry_msgs::PointStamped loc_pos);
void cb_gps_position(const sensor_msgs::NavSatFix gps_po);
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);
void get_imu(const sensor_msgs::Imu imu);

// 영상 녹화 및 로그 경로
bool test_bool = get_time();
bool test_bool_1 = get_src();
ofstream outfile(src_log);

int main(int argc, char **argv){
  ros::init(argc, argv, "save_log_gps");
  ros::NodeHandle nh;

  // 쓰레드 종료할지 정하기 위한 변수
  is_ctrl_c = false;

  cout << "save_log_gps start" << endl;
  
  // 로그 항목
  string csv_name = "Time,Time_unix,\
  gps_health,local_position_x,local_position_y,local_position_z,\
  position_stat,position_lat,position_lon,position_alt,\
  vel_x,vel_y,vel_z";
  outfile << csv_name << endl;
  
  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = nh.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_gps_hlth = nh.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_local_pos = nh.subscribe("dji_sdk/local_position", 1000, cb_gps_local_position);
  ros::Subscriber sub_gps_position = nh.subscribe("dji_sdk/gps_position", 1000, cb_gps_position);
  ros::Subscriber sub_gps_vel = nh.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);

  ros::Rate loop_rate(50);
  
  auto save_log = []()
	{
    cout << "start save log" << endl;
    ros::Rate loop_rate_save(100);
		while(!is_ctrl_c){
      // 시간 초기화
      get_time();
      string sec_string = to_string(sec+m_sec);
      if(sec+m_sec < 10){
        sec_string = "0" + sec_string.substr(0,5);
      }
      else{
        sec_string = sec_string.substr(0,6);
      }

      if(imu_ori_y == 0){
        continue;
      }

      string time_now = "Time : " + to_string(hour) + ":" + to_string(minute) + ":" + sec_string;
      
      outfile << time_now << "," << ros::Time::now() << ",";
      outfile << gps_health << ",";
      outfile << loc_po_x << "," << loc_po_y << "," << loc_po_z << ",";
      outfile << gps_po_stat << "," << gps_po_lat << "," << gps_po_lon << ",";
      outfile << gps_po_alt << ",";
      outfile << vel_x << "," << vel_y << "," << vel_z << endl;
      
      loop_rate_save.sleep();
      // ros::Duration(0.009).sleep();
    }
	};

  auto start_get_sensor = []()
  {
    cout << "start ros spin" << endl;
    ros::spin();
    is_ctrl_c = true;
  };
  
  std::thread t1 = std::thread(save_log);
  std::thread t2 = std::thread(start_get_sensor);
  t1.join();
  t2.join();

  return 0;
}

bool get_time(){
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

  // 시간 기록하기 위함
  double secs = ros::Time::now().toSec();
  double msec;
  secs = secs - (int)secs;
  msec = secs * 1000;
  msec = msec - (int)msec;
  msec = msec / 1000;
  secs = secs - msec;
  m_sec = secs; // global

  return true;
}

bool get_src(){
  if(!is_test){
    string vid_name = to_string(month) + "_" + to_string(day) + "_" + to_string(hour) + "_" + to_string(minute) + "_" + to_string(sec);
    src_log = "/home/aims/Desktop/vid_output/" + vid_name + "_gps.csv";
  }
  else{
    src_log = "/home/aims/Desktop/vid_output/log_gps.csv";
  }
  return true;
}

void cb_gps_position(const sensor_msgs::NavSatFix gps_po){
    gps_po_stat = gps_po.status.status;
    gps_po_lat = gps_po.latitude;
    gps_po_lon = gps_po.longitude;
    gps_po_alt = gps_po.altitude;
}

void cb_gps_local_position(const geometry_msgs::PointStamped loc_pos){
    loc_po_x = loc_pos.point.x;
    loc_po_y = loc_pos.point.y;
    loc_po_z = loc_pos.point.z;
}

void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel){
    vel_x = vel.vector.x;
    vel_y = vel.vector.y;
    vel_z = vel.vector.z;
}

void cb_gps_health(const std_msgs::UInt8 gps_h){gps_health = gps_h.data;}
void get_imu(const sensor_msgs::Imu imu){imu_ori_y = imu.orientation.y;}