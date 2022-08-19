#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"
#include "ros/ros.h"
#include "dji_sdk/VOPosition.h"

// // multi thread
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <string>

#include <sstream>
#include <time.h>

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

using namespace std;
// // 전역 변수 정의
bool is_ctrl_c;

// imu
float imu_ori_x, imu_ori_y, imu_ori_z, imu_ori_w;
float imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z;
float imu_lin_acc_x, imu_lin_acc_y, imu_lin_acc_z;
//acceleration_ground_fuse
float acc_x; // 오른쪽
float acc_y; // 앞 - 맞나? 동쪽 북쪽이었던거 같은데
float acc_z; // 상
// gps 관련
// GPS - GPS_health
float gps_health;
// GPS - gps position
float gps_po_stat, gps_po_lat, gps_po_lon, gps_po_alt;
// GPS - gps velocity
float vel_x, vel_y, vel_z;
// VOPosition
float vo_x, vo_y, vo_z;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// bool get_time();
void cb_acc_ground(const geometry_msgs::Vector3Stamped acc);
// gps 관련
void cb_gps_health(const std_msgs::UInt8 gps_h);
void cb_gps_position(const sensor_msgs::NavSatFix gps_po);
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);
void cb_voposition(const dji_sdk::VOPosition pos);

int main(int argc, char **argv){

  // 쓰레드 종료할지 정하기 위한 변수
  is_ctrl_c = false;
  
  ros::init(argc, argv, "calculate velocity");
  ros::NodeHandle nh;
  
  ros::Subscriber sub_imu = nh.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_acc_ground = nh.subscribe("dji_sdk/acceleration_ground_fused", 1000,cb_acc_ground);
  ros::Subscriber sub_gps_hlth = nh.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_position = nh.subscribe("dji_sdk/gps_position", 1000, cb_gps_position);
  
  ros::Rate loop_rate(50);

  auto save_log = []()
	{
    cout << "start calculate velocity" << endl;
    ros::Rate loop_rate_save(100);
		while(!is_ctrl_c){

      if(imu_ori_y == 0){
        continue;
      }

      EulerAngles angles;
      Quaternion q;
      q.w = imu_ori_w;
      q.x = imu_ori_x;
      q.y = imu_ori_y;
      q.z = imu_ori_z;
      
      angles = ToEulerAngles(q);
      float theta = angles.yaw;
      float vel_f = vel_x*cos(theta) + vel_y*sin(theta);
      float vel_r = vel_x*sin(theta) - vel_y*cos(theta);

      float theta_deg = theta * 3.14159265358979 / 180;
      
      cout << "angle is : " << theta_deg << "\n" << endl;
      cout << "front speed is : " << vel_f << endl;
      cout << "right speed is : " << vel_r << "\n" << endl;
      cout << "lat : " << gps_po_lat << endl;
      cout << "lon : " << gps_po_lon << "\n" << endl;
      cout << "North m : " << vo_x << endl;
      cout << "East m : " << vo_y << "\n" << endl;

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
}

void cb_acc_ground(const geometry_msgs::Vector3Stamped acc){
  acc_x = acc.vector.x;
  acc_y = acc.vector.y;
  acc_z = acc.vector.z;
}


void cb_gps_position(const sensor_msgs::NavSatFix gps_po){
    gps_po_stat = gps_po.status.status;
    gps_po_lat = gps_po.latitude;
    gps_po_lon = gps_po.longitude;
    gps_po_alt = gps_po.altitude;
}

void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel){
    vel_x = vel.vector.x;
    vel_y = vel.vector.y;
    vel_z = vel.vector.z;
}

void cb_gps_health(const std_msgs::UInt8 gps_h){gps_health = gps_h.data;}

void cb_voposition(const dji_sdk::VOPosition pos){
  cout << pos.x << endl;
  vo_x = pos.x;
  vo_y = pos.y;
  vo_z = pos.z;
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