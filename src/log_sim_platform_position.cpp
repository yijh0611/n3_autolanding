#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/PointStamped.h"
// #include "sensor_msgs/NavSatFix.h"
// #include "geometry_msgs/Vector3Stamped.h"
// #include "dji_sdk/DroneArmControl.h"
// 이미지
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
// #include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
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
// 이미지 출력을 위한 임시 코드
// #include "sensor_msgs/CameraInfo.h"
#include <time.h>
#include "dji_sdk/VOPosition.h"

// 수정 사항 및 코드 특징
// log_sim_platform_position - 모든 정보를 저장하는건 아니고,
// 플랫폼, 드론, 시뮬레이션 태그 정보, IMU orientation 만 저장
// 멀티쓰레드로 로그 저장

// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

using namespace std;
// 전역 변수 정의
bool is_test = false; // 이거 true 일때는 연습
bool is_ctrl_c;
// 태그 위치
float x_tf, y_tf, z_tf;
float tf_rot_x, tf_rot_y, tf_rot_z, tf_rot_w;
// 현재 시간
int year, month, day, hour, minute, sec;
double m_sec;
// imu // 수정 필요
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;
float imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z;
float imu_lin_acc_x, imu_lin_acc_y, imu_lin_acc_z;

// VOPosition
float vo_x, vo_y, vo_z;

// Target(Platform 위치)
// float target_x, target_y, vel_platform_r_abs, vel_platform_f_abs;
// float move_right, dist_r, vel_platform_r, move_right_real, vel_r_drone;
float theta, theta_deg, vo_x_cal;

// platform 위치
float platform_e, platform_n;
float target_x, target_y; // 시뮬레이션에서 알려주는 플랫폼의 진짜 위치
float kp = 0;
float ki = 0;
float kd = 0;
float d_x = 0;
float v_x = 0;
float x_i = 0;
float kixi = 0;
float dt = 0;

// 영상 녹화 및 로그
string src_vid, src_log;
// 영상 프레임 낮추기 위한 변수
float is_save_frame = 0;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
bool get_time();
bool get_src();
// gps 관련
void cb_voposition(const dji_sdk::VOPosition pos); // 위치
// Platform 위치
void callback_tf_sim(const std_msgs::Float64MultiArray tf_sim); // Tag 정보 simulation
void callback_pid(const std_msgs::Float64MultiArray pid);

float get_yaw(float w, float x, float y, float z);
float fru_to_enu(float f, float r, float theta);

// // 영상 녹화 및 로그 경로
bool test_bool = get_time();
bool test_bool_1 = get_src();
ofstream outfile(src_log);

int main(int argc, char **argv){

  // 쓰레드 종료할지 정하기 위한 변수
  is_ctrl_c = false;
  
  // 로그 항목
  string csv_name = "Time,Time_unix,tag_right,tag_down,tag_height,drone_east,drone_north,drone_height,\
  platform_east_real,platform_north_real,platform_east,platform_north,kp,ki,kd,dx,vx,xi,ki*xi,dt";
  outfile << csv_name << endl;

  ros::init(argc, argv, "log_sim_platform_position");
  ros::NodeHandle nh;
  
  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = nh.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_vo_pos = nh.subscribe("dji_sdk/vo_position", 1000, cb_voposition);
  ros::Subscriber sub_platform = nh.subscribe("platform_sim", 1000, callback_tf_sim);
  ros::Subscriber sub_pid = nh.subscribe("pid_sim", 1000, callback_pid);

  ros::Rate loop_rate(50);

  auto save_log = []()
	{
    cout << "Start save log" << endl;
    ros::Rate loop_rate_save(100);
		while(!is_ctrl_c){
      // cout << "Save log" << endl;
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
      // outfile << input_f << "," << input_l << "," << input_u << "," << input_cc << "," << input_q << ",";
      outfile << x_tf << "," << y_tf << "," << -1 * z_tf << ",";
      // outfile << imu_ori_x << "," << imu_ori_y << "," << imu_ori_z << "," << imu_ori_w << ",";
      
      outfile << vo_x << "," << vo_y << "," << vo_z << ",";
      outfile << target_x << "," << target_x << ",";
      outfile << platform_e << "," << platform_n << ",";
      outfile << kp << "," << ki << "," << kd << ",";
      outfile << d_x << "," << v_x << "," << x_i << ",";
      outfile << kixi << "," << dt << endl;


      // // tf 초기화
      // x_tf = 0;y_tf = 0; z_tf = 0;
      // tf_rot_x = 0; tf_rot_y = 0; tf_rot_z = 0; tf_rot_w = 0;
      // // 입력 명령 초기화
      // input_f = 0; input_l = 0; input_u = 0; input_cc = 0; input_q = 0;
      // // GPS 초기화 - 이거 하면 꼬여서 값이 있는데도 로그로 저장이 안된다.
      // gps_health = 0; loc_po_x = 0; loc_po_y = 0; loc_po_z = 0; gps_po_stat =0;
      // gps_po_lat = 0; gps_po_lon = 0; gps_po_alt = 0; vel_x = 0; vel_y = 0; vel_z = 0;

      // ros::Duration(0.009).sleep();
      loop_rate_save.sleep();
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

void get_imu(const sensor_msgs::Imu imu)
{
  imu_ori_x = imu.orientation.x;
  imu_ori_y = imu.orientation.y;
  imu_ori_z = imu.orientation.z;
  imu_ori_w = imu.orientation.w;

  imu_ang_vel_x = imu.angular_velocity.x;
  imu_ang_vel_y = imu.angular_velocity.y;
  imu_ang_vel_z = imu.angular_velocity.z;

  imu_lin_acc_x = imu.linear_acceleration.x;
  imu_lin_acc_y = imu.linear_acceleration.y;
  imu_lin_acc_z = imu.linear_acceleration.z;

  // cout << imu << endl;
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

  return true;
}

bool get_src(){
  if(!is_test){
    string vid_name = to_string(month) + "_" + to_string(day) + "_" + to_string(hour) + "_" + to_string(minute) + "_" + to_string(sec);
    src_vid = "/home/aims/Desktop/vid_output/" + vid_name + "sim.avi";
    src_log = "/home/aims/Desktop/vid_output/" + vid_name + "sim.csv";
  }
  else{
    src_vid = "/home/aims/Desktop/vid_output/test.avi";
    src_log = "/home/aims/Desktop/vid_output/log.csv";
  }
  return true;
}

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_x = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_y = pos.x;
  vo_z = pos.z;
}

float get_yaw(float w, float x, float y, float z){
  float yaw_x = 2 * (w * z + x * y);
  float yaw_y = 1 - 2 * (y * y + z * z);
  float yaw = atan2(yaw_x, yaw_y);

  return yaw;
}

float fru_to_enu(float f, float r, float theta){
  // 데이터는 flu 이기 때문에, 이거 쓸 일은 별로 없어보인다.
  float e = r*sin(theta) + f*cos(theta);
  float n = -1*r*cos(theta) + f*sin(theta);

  return e, n;
}

void callback_tf_sim(const std_msgs::Float64MultiArray tf_sim){
  x_tf = tf_sim.data[0];
  y_tf = tf_sim.data[1];
  z_tf = tf_sim.data[2];
  target_x = tf_sim.data[3];
  target_y = tf_sim.data[4];

  float yaw = get_yaw(imu_ori_w, imu_ori_x, imu_ori_y, imu_ori_z);
  // cout << yaw << endl;
  float d_e, d_n;
  float f_tf = -1 * y_tf;
  d_e, d_n = fru_to_enu(f_tf, x_tf, yaw);

  platform_e = vo_x + d_e;
  platform_n = vo_y + d_n;
}

void callback_pid(const std_msgs::Float64MultiArray pid){
  kp = pid.data[0];
  ki = pid.data[1];
  kd = pid.data[2];
  d_x = pid.data[3];
  v_x = pid.data[4];
  x_i = pid.data[5];
  kixi = pid.data[6];
  dt = pid.data[7];
}