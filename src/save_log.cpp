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
// save_log - 정보들 다 받아와서 저장하는 코드
// 멀티쓰레드로 로그 저장
// 영상 받고 시간 적은 뒤 저장
// 센서 정보 받아오기
// GPS정보도 같이 저장 및 atti 제거

// 아직 작업 중인 부분
// 제어 입력 받아오기
// 제어입력은 어떻게 보내고 받고, 분석할건지 정해야 됨.
// 태그 정보 받아야 됨

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
//cb_battery;
float batt_volt; // voltage
float batt_curr; // current
float batt_cap; // capacity
float batt_per; // percentage
//acceleration_ground_fuse
float acc_x; // 오른쪽
float acc_y; // 전진
float acc_z; // 상
//cb_atti -> ?
float atti_x; // 쿼터니언인데, 복잡해서 나중에 다시 확인 !!
float atti_y;
float atti_z;
float atti_w;
//angular_velocity_fused
float ang_vel_x; // 값이 너무 작게 변해서 로그 뽑아보면서 뭔지 확인하는게 나을 듯 !!
float ang_vel_y;
float ang_vel_z;
// Drone control input
float input_f = 0;
float input_l = 0;
float input_u = 0;
float input_cc = 0;
float input_q = 0; // 이거는 값이 75로 출력되는데, 뭔지 잘 모르겠어서 q로 했다.
// gps 관련
// GPS - GPS_health
float gps_health;
// GPS - local position
float loc_po_x, loc_po_y, loc_po_z;
// GPS - gps position
float gps_po_stat, gps_po_lat, gps_po_lon, gps_po_alt;
// GPS - gps velocity
float vel_x, vel_y, vel_z;
// VOPosition
float vo_x, vo_y, vo_z;
// Kalman filter
float kal_f, kal_r;
float kal_v_f, kal_v_r;
float time_kalman;
float is_tag, is_tag_lost;
float kal_tf_x, kal_tf_y, kal_tf_z;
float kal_dt, tag_vx, tag_vy, tag_vz;
// 데이터 확인용
float d_x,v_x,d_y,v_y,dt_data;
float target_input_f, target_input_l;
float kp_control, kd_control;
float pid_front, pid_right;
float spd_constant;
float kal_r_vel_tmp, kal_f_vel_tmp;

// 영상 녹화 및 로그
string src_vid, src_log;
// 영상 프레임 낮추기 위한 변수
float is_save_frame = 0;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
void callback_tf(const tf2_msgs::TFMessage msg3);
void get_img(const sensor_msgs::ImageConstPtr& img_ros);
bool get_time();
bool get_src();
void cb_battery(const sensor_msgs::BatteryState battery);
void cb_acc_ground(const geometry_msgs::Vector3Stamped acc);
void cb_atti(const geometry_msgs::QuaternionStamped atti);
void cb_ang_vel(const geometry_msgs::Vector3Stamped ang_vel);
void cb_joy(const sensor_msgs::Joy joy);
// gps 관련
void cb_gps_health(const std_msgs::UInt8 gps_h);
void cb_gps_local_position(const geometry_msgs::PointStamped loc_pos);
void cb_gps_position(const sensor_msgs::NavSatFix gps_po);
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);
void cb_voposition(const dji_sdk::VOPosition pos); // 위치
// Kalman filter
void cb_kalman(const std_msgs::Float64MultiArray kal);
// Distance data
void cb_distance(const std_msgs::Float64MultiArray dist);

// 영상 녹화 및 로그 경로
bool test_bool = get_time();
bool test_bool_1 = get_src();
ofstream outfile(src_log);

cv::VideoWriter videoWriter;
float videoFPS = 30; // 원래는 100인데, 30으로 줄였음. 100
int videoWidth = 640;
int videoHeight = 480;

int main(int argc, char **argv){

  // 쓰레드 종료할지 정하기 위한 변수
  is_ctrl_c = false;
  
  // 로그 항목
  string csv_name = "Time,Time_unix,input_f,input_l,input_u,input_cc,input_q,tag_x_tf,tag_y_tf,tag_z_tf,\
  tf_rot_x, tf_rot_y, tf_rot_z, tf_rot_w,\
  imu_orientation_x,imu_orientation_y,imu_orientation_z,imu_orientation_w,imu_angle_vel_x,\
  imu_angle_vel_y,imu_angle_vel_z,imu_linear_acc_x,imu_linear_acc_y,imu_linear_acc_z,\
  acc_right,acc_front,acc_up,ang_vel_x,ang_vel_y,ang_vel_z,\
  gps_health,local_position_x,local_position_y,local_position_z,\
  position_stat,position_lat,position_lon,position_alt,\
  vel_x,vel_y,vel_z,\
  batt_volt,batt_curr,batt_cap,batt_percentage,\
  vo_x,vo_y,v_z,kal_r,kal_f,kal_v_r,kal_vel_f,time_kal,is_tag,is_tag_lost,\
  kal_tf_x,kal_tf_y,kal_tf_z,dt,v_x,v_y,x_z,tag_d_x,tag_v_x,tag_d_y,tag_v_y,dt_pid,\
  PID_front,PID_right,target_input_f,target_input_l,\
  kp_control,kd_control,spd_constant,kal_r_vel_tmp,kal_f_vel_tmp";
  outfile << csv_name << endl;

  ros::init(argc, argv, "save_log");
  ros::NodeHandle nh;
  
  // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_imu = nh.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_tf = nh.subscribe("tf", 1000, callback_tf);
  ros::Subscriber sub_battery = nh.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  ros::Subscriber sub_acc_ground = nh.subscribe("dji_sdk/acceleration_ground_fused", 1000,cb_acc_ground);
  // ros::Subscriber sub_atti = nh.subscribe("dji_sdk/attitude", 1000, cb_atti);
  ros::Subscriber sub_ang_vel = nh.subscribe("dji_sdk/angular_velocity_fused", 1000, cb_ang_vel);
  ros::Subscriber sub_joy = nh.subscribe("dji_sdk/flight_control_setpoint_generic", 1000, cb_joy);
  ros::Subscriber sub_gps_hlth = nh.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_local_pos = nh.subscribe("dji_sdk/local_position", 1000, cb_gps_local_position);
  ros::Subscriber sub_gps_position = nh.subscribe("dji_sdk/gps_position", 1000, cb_gps_position);
  ros::Subscriber sub_gps_vel = nh.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);
  ros::Subscriber sub_vo_pos = nh.subscribe("dji_sdk/vo_position", 1000, cb_voposition);
  ros::Subscriber sub_kal = nh.subscribe("kalmanfilter", 1000, cb_kalman);
  ros::Subscriber sub_pid_data = nh.subscribe("distance", 1000, cb_distance);

  // 영상 녹화
  videoWriter.open(src_vid, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), videoFPS , cv::Size(videoWidth, videoHeight), true);
  // 영상 수신
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("camera_rect/image_rect", 1, get_img);

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
      outfile << input_f << "," << input_l << "," << input_u << "," << input_cc << "," << input_q << ",";
      outfile << x_tf << "," << y_tf << "," << z_tf << ",";
      outfile << tf_rot_x << "," << tf_rot_y << "," << tf_rot_z << "," << tf_rot_w << ",";
      outfile << imu_ori_x << "," << imu_ori_y << "," << imu_ori_z << "," << imu_ori_w << ",";
      outfile << imu_ang_vel_x << "," << imu_ang_vel_y << "," << imu_ang_vel_z << ",";
      outfile << imu_lin_acc_x << "," << imu_lin_acc_y << "," << imu_lin_acc_z << ",";
      outfile << acc_x << "," << acc_y << "," << acc_z << ",";
      // outfile << atti_x << "," << atti_y << "," << atti_z << "," << atti_w << ",";
      outfile << ang_vel_x << "," << ang_vel_y << "," << ang_vel_z << ",";

      outfile << gps_health << ",";
      
      outfile << loc_po_x << "," << loc_po_y << "," << loc_po_z << ",";
      outfile << gps_po_stat << "," << gps_po_lat << "," << gps_po_lon << ",";
      outfile << gps_po_alt << ",";
      outfile << vel_x << "," << vel_y << "," << vel_z << ",";

      outfile << batt_volt << "," << batt_curr << "," << batt_cap << "," << batt_per << ",";
      outfile << vo_x << "," << vo_y << "," << vo_z << ",";
      outfile << kal_r << "," << kal_f << "," << kal_v_r << ",";
      outfile << kal_v_f << "," << time_kalman << "," << is_tag << ",";
      outfile << is_tag_lost << "," << kal_tf_x << "," << kal_tf_y << ",";
      outfile << kal_tf_z << "," << kal_dt << "," << tag_vx << ",";
      outfile << tag_vy << "," << tag_vz << "," << d_x << ",";
      outfile << v_x << "," << d_y << "," << v_y << ",";
      outfile << dt_data << "," << pid_front << "," << pid_right << ",";
      outfile << target_input_f << "," << target_input_l << ",";
      outfile << kp_control << "," << kd_control << ",";
      outfile << spd_constant << "," << kal_r_vel_tmp << ",";
      outfile << kal_f_vel_tmp << "," << endl;

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

void get_img(const sensor_msgs::ImageConstPtr& img_ros){// const sensor_msgs::Image
  try{
    
    // 시간 기록하기 위함
    double secs = ros::Time::now().toSec();
    double msec;
    secs = secs - (int)secs;
    msec = secs * 1000;
    msec = msec - (int)msec;
    msec = msec / 1000;
    secs = secs - msec;
    m_sec = secs; // global

    get_time();

    // put text
    string sec_string = to_string(sec+m_sec);
    if(sec+m_sec < 10){
      sec_string = "0" + sec_string.substr(0,5);
    }
    else{
      sec_string = sec_string.substr(0,6);
    }

    string myText = "Time : " + to_string(hour) + ":" + to_string(minute) + ":" + sec_string;
        
    cv::Point myPoint;
    myPoint.x = 10;
    myPoint.y = 40;
    /// Font Face
    int myFontFace = 2;
    /// Font Scale
    double myFontScale = 1.0;
    cv::putText(cv_bridge::toCvShare(img_ros, "bgr8")->image, myText, myPoint, myFontFace, myFontScale, cv::Scalar(255, 0, 0)); // cv::Scalar::all(255)

    myPoint.y = 80;
    if(is_tag_lost == 1){
      myText = "No Tag";
    }
    else{
      myText = "Tag found";
    }
    cv::putText(cv_bridge::toCvShare(img_ros, "bgr8")->image, myText, myPoint, myFontFace, myFontScale, cv::Scalar(255, 0, 0));

    // // 이미지 저장
    // cv::imwrite(src_img,cv_bridge::toCvShare(img_ros, "bgr8")->image);

    // 영상 녹화
    videoWriter << cv_bridge::toCvShare(img_ros, "bgr8")->image;

    // cv::imshow("view", cv_bridge::toCvShare(img_ros, "bgr8")->image);
    
    // cv::waitKey(30);
    // is_save_frame = 0;
    // }
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_ros->encoding.c_str());
  }
}

void callback_tf(const tf2_msgs::TFMessage msg3)
{ // 값 저장하는 코드
  x_tf = msg3.transforms[0].transform.translation.x;
  y_tf = msg3.transforms[0].transform.translation.y; //z_tf는 없어도 되는건가?
  z_tf = msg3.transforms[0].transform.translation.z;

  tf_rot_x = msg3.transforms[0].transform.rotation.x;
  tf_rot_y = msg3.transforms[0].transform.rotation.y;
  tf_rot_z = msg3.transforms[0].transform.rotation.z;
  tf_rot_w = msg3.transforms[0].transform.rotation.w;
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
    src_vid = "/home/aims/Desktop/vid_output/" + vid_name + ".avi";
    src_log = "/home/aims/Desktop/vid_output/" + vid_name + ".csv";
  }
  else{
    src_vid = "/home/aims/Desktop/vid_output/test.avi";
    src_log = "/home/aims/Desktop/vid_output/log.csv";
  }
  return true;
}

void cb_battery(const sensor_msgs::BatteryState battery){
  batt_volt = battery.voltage;
  batt_curr = battery.current;
  batt_cap = battery.capacity;
  batt_per = battery.percentage;
}

void cb_acc_ground(const geometry_msgs::Vector3Stamped acc){
  acc_x = acc.vector.x;
  acc_y = acc.vector.y;
  acc_z = acc.vector.z;
}

void cb_atti(const geometry_msgs::QuaternionStamped atti){
  atti_x = atti.quaternion.x;
  atti_y = atti.quaternion.y;
  atti_z = atti.quaternion.z;
  atti_w = atti.quaternion.w;
}

void cb_ang_vel(const geometry_msgs::Vector3Stamped ang_vel){
  ang_vel_x = ang_vel.vector.x;
  ang_vel_y = ang_vel.vector.y;
  ang_vel_z = ang_vel.vector.z;
}

void cb_joy(const sensor_msgs::Joy joy){
  input_f = joy.axes[0];
  input_l = joy.axes[1];
  input_u = joy.axes[2];
  input_cc = joy.axes[3];
  input_q = joy.axes[4];
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

void cb_voposition(const dji_sdk::VOPosition pos){ // 이거 0.1Hz 라서 사용하기 힘들거 같다.
  // cout << pos.x << endl;
  vo_x = pos.y; // y가 동쪽인데, 코드를 x를 동쪽으로 짜서 이렇게 바꿨음.
  vo_y = pos.x;
  vo_z = pos.z;
}

void cb_kalman(const std_msgs::Float64MultiArray kal){
  kal_f = kal.data[0];
  kal_r = kal.data[1];
  kal_v_f = kal.data[2];
  kal_v_r = kal.data[3];
  time_kalman = kal.data[4];
  is_tag = kal.data[5];
  is_tag_lost = kal.data[6];
  kal_tf_x = kal.data[7];
  kal_tf_y = kal.data[8];
  kal_tf_z = kal.data[9];  
  kal_dt = kal.data[10];
  tag_vx = kal.data[11];
  tag_vy = kal.data[12];
  tag_vz = kal.data[13];
}

void cb_distance(const std_msgs::Float64MultiArray dist){
  d_x = dist.data[0];
  v_x = dist.data[1];
  d_y = dist.data[2];
  v_y = dist.data[3];
  dt_data = dist.data[4];
  pid_right = dist.data[5];
  pid_front = dist.data[6];
  target_input_f = dist.data[7];
  target_input_l = dist.data[8];
  kp_control = dist.data[9];
  kd_control = dist.data[10];
  spd_constant = dist.data[11];
  kal_r_vel_tmp = dist.data[12];
  kal_f_vel_tmp = dist.data[13];
}
