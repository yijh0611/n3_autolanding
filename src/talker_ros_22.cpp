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
#include "geometry_msgs/Transform.h" // 시뮬레이션

// multi thread
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <string>

#include <sstream>
// 이미지 출력을 위한 임시 라이브러리
#include "sensor_msgs/CameraInfo.h"
#include <time.h>
#include<math.h>
// Time in ms(1/1000s)
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
// 코드 이름 : talker_ros_22
// PID 계수 P:0.7, D:0.07
// 정지한 태그에 착륙하지는 않지만, 다가가는 코드

// 정지한 태그위에 착륙하는 코드 추가하려고 함 - 만들려다가 말았음. - land_1으로 다시 만드는 중
// 태그보다 약간 뒤에서 착륙하게 코드를 만들면, 바람에 밀리는 상황을 알 수 없기 때문에
// 작은 태그를 보고 착륙하게 만드는게 좋을 것 같다.
// 우선 호버링 먼저 해보고 
// 태그 여러개 있을때 구분을 하는 코드도 있어야 할 듯
// 칼만필터에서 얻은 정보 이용해서 제어 + 고도는 3.5m로 유지
// move함수안에 0.02초 기다리는거 있는데, 빼도 괜찮은지 확인 !!
// Tag 안보이다가 보이면, 0.5초정도 간격을 두고 Kalman filter speed를 서서히 늘리는 코드 추가
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
// 드론 속력 파악
float drone_vel_e, drone_vel_n;
// Tag position
float r_prev = 0;
float f_prev = 0;
float z_prev = 0;
float dt = 0;
// Tag 정보 수신이 되는지 확인
bool is_tag_signal = true; // 시뮬이 아니면 true. 원래는 false였고, simulation인지 아닌지 잘 판단할 수 있게 수정하기 !!

// PID 계수
float kp = 0.7; 
float kd = 0.07;
float ki = 0; // 필요한가?
float max_move_spd = 5;
// // 속도제어 PD 계수 - 값은 함수 안에서 정의
float kp_control, kd_control;

// 칼만필터
float kal_r,kal_f,kal_r_vel,kal_f_vel,kal_time,kal_is_tag,kal_is_tag_lost;
bool is_kalman = true;
// 태그 위치
float x_tf, y_tf, z_tf;
string tf_name;
int count_tf = 0;
bool big_tag = true;
double time_prev;
float gimbal_down;
bool is_gimbal_down = false; // 밖에서 할때는 false, 안에서 할때는 true
bool is_boost = false; // true 이면 P계수 변경
float boost_rate = 2;

// 드론 제어해도 되는지 판단
bool is_drone_move = false; // 이거는 안전검사 끝나면 알아서 바뀜
float spd_r, spd_f, spd_u, spd_y;
float spd_r_prev, spd_f_prev, spd_u_prev, spd_y_prev;
float drone_vel_f_prev = 0;
float drone_vel_r_prev = 0;
// tf 정보

// 시뮬레이션이면 안전검사 스킵
bool is_sim = false; // false : 실제, true : 시뮬레이션 - 이거만 바꾸면 됨

// 드론 제어 권한
ros::ServiceClient sdk_ctrl_authority_service;

// 함수 정의
void get_imu(const sensor_msgs::Imu msg);
// gps 관련
void cb_gps_health(const std_msgs::UInt8 gps_h);
void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel);

void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void move(float cont[], ros::NodeHandle n, ros::Publisher control_pub);
void cb_battery(const sensor_msgs::BatteryState battery);
void cb_kalman(const std_msgs::Float64MultiArray kal);
void callback_tf(const tf2_msgs::TFMessage msg3); // Tag 정보 - 거리 기반 PID
void callback_tf_2(const geometry_msgs::Transform tag_tmp); // Tag 정보 simulation
void get_time();
float get_yaw(float w, float x, float y, float z);
float enu_to_fru_f(float e, float n, float y);
float enu_to_fru_r(float e, float n, float y);

std_msgs::Float64MultiArray gim;
std_msgs::Float64MultiArray distance_data;

int main(int argc, char **argv){
  if(is_sim == true){
    is_gimbal_down = true;
  }
  else{
    is_gimbal_down = false;
  }
  if(is_gimbal_down){
    gimbal_down = 76;
  }
  else{
    gimbal_down = 56;
  }
  
  is_run_thread = true;
  ros::init(argc, argv, "talker_ros_22");
  ros::NodeHandle n;

  // subscriber
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, get_imu);
  ros::Subscriber sub_gps_hlth = n.subscribe("dji_sdk/gps_health", 1000, cb_gps_health);
  ros::Subscriber sub_gps_vel = n.subscribe("dji_sdk/velocity", 1000, cb_gps_velocity);
  ros::Subscriber sub_battery = n.subscribe("dji_sdk/battery_state", 1000, cb_battery);
  ros::Subscriber sub_kalman = n.subscribe("kalmanfilter", 1000, cb_kalman);
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
  // ros::Subscriber sub_tf_2 = n.subscribe("tf_2", 1000, callback_tf_2);
  
  // publisher
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  ros::Publisher distance_pub = n.advertise<std_msgs::Float64MultiArray>("distance", 10);
 
  // 안전 검사
  if (is_sim){
    cout << "Simulation" << endl;
  }
  else{
      
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
  }

  // // simulation
  // chk_start = 0;
  // while(is_tag_signal == false){ // GPS 수신 안되면 일단 넘어가는데, 이거도 수정해야 할 듯.!!
  //   ros::Duration(0.1).sleep();
  //   chk_start += 1;
  //   if (chk_start > 100){
  //     ROS_ERROR("No Tag singal for 10 Sec!");
  //     ROS_ERROR("Simulation");
  //     // ros::Subscriber sub_tf_2 = n.subscribe("tf_2", 1000, callback_tf_2);
  //     break;
  //   }
  //   ros::spinOnce();
  // }

  // if (is_tag_signal == false){
  //   ros::Subscriber sub_tf_2 = n.subscribe("tf_2", 1000, callback_tf_2);
  //   ros::spin();
  // }


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

  for(int i=0; i < 15; i++){ // 5까지 하면 arr[4]까지 쓸 수 있음.
    distance_data.data.push_back(0.0);
  }
  

  // // 계수 입력 받기
  if (is_tag_signal){ // 시뮬레이션이 아닐 때
    cout << "Maximum speed" << endl;
    cin >> max_move_spd;
  }

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
    // if (is_tag_signal == false){
    //   cout << "Start TF_2" << endl;
    //   ros::NodeHandle nh;
    //   ros::Subscriber sub_tf_2 = nh.subscribe("tf_2", 1000, callback_tf_2);
    // }
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
    double tag_time = ros::Time::now().toSec();
    struct timeval time_now{};
    time_t msecs_tag_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

    int count_tag_lost = 0;

    while(is_run_thread){ // 이게 수정된 제어 코드
      count_tag_lost += 1;
      if(ros::Time::now() - time_no_tag > ros::Duration(time_limit)){
        move_front = 0;
        move_right = 0; // 안해도 되는데, 로그 저장할 때 있으면 좋을 듯.
        is_drone_move = false;
        if(count_tag_lost % 100 == 0){
          ROS_ERROR("Tag is lost");
        }
        tag_time = ros::Time::now().toSec();
      }

      // 태그 거리기반 PID
      if(tag_x_tmp != x_tf){ // 태그 정보가 바뀌었을때
        cout << "Distance : " << z_tf << endl;
        // ROS_INFO(z_tf);
        if(count_tag_lost % 100 == 0){
          ROS_INFO("Tag is found");
          count_tag_lost = 0;
        }

        float d_x = x_tf;
        float d_y = -1 * y_tf;
        
        dt = ros::Time::now().toSec() - time_prev;
        float d_dx = d_x - d_x_prev;
        float d_dy = d_y - d_y_prev;
        d_x_prev = d_x;
        d_y_prev = d_y;
        time_prev = ros::Time::now().toSec();
        
        float v_x = 0;
        float v_y = 0;

        if(ros::Time::now().toSec() - tag_time > 0.2){ // 태그를 찾고 0.2동안은 속력 0이라고 생각 - 10Hz 정도로 Tag 정보가 들어오기 때문에 0.2초면 괜찮을 듯 하다.
          v_x = d_dx / dt;
          v_y = d_dy / dt;
        }

        float kp_l = kp;
        float kp_f = kp;

        if (is_boost){ // 나중에 제거하기
          kp_l = 1 + fabs(y_tf / z_tf) * 2 * (boost_rate - 1);
          kp_f = 1 + fabs(x_tf / z_tf) * 3 * (boost_rate - 1);
        }

        // if(d_x < 0.5 && d_y < 0.5){ // 가까울때 움직이지 않는다 - 조건문 좀 더 추가해서 태그가 다를때 어떻게 할지 등을 넣으면 좋을 듯.
        //   move_right = 0;
        //   move_front = 0;
        // }
        // else{ // 멀리 있을때만 움직인다.
          // move_right = kp_l * d_x + kd * v_x;
          // move_front = kp_f * d_y + kd * v_y;
        // }
        move_right = kp_l * d_x + kd * v_x;
        move_front = kp_f * d_y + kd * v_y;

        distance_data.data[0] = d_x;
        distance_data.data[1] = v_x;
        distance_data.data[2] = d_y;
        distance_data.data[3] = v_y;
        distance_data.data[4] = dt;
        distance_data.data[5] = move_right;
        distance_data.data[6] = move_front;
        // distance_pub.publish(distance_data); - 뒤에서 Publish

        // 여기까지 거리 기반

        // 칼만필터
        // if(kal_is_tag_lost == 0){ // 태그 놓치지 않았을 때
        float spd_constant = 0; // 1, 0.85 !! 원래는 1 - 지금 0이라서 칼만필터를 이용한 속도제어는 작동하지 않는다.
        float spd_constant_tmp = spd_constant;
        // if(ros::Time::now().toSec() - tag_time < 0.5){
        // gettimeofday(&time_now, nullptr);
        // time_t msecs_tag_time_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
        // if((float)msecs_tag_time_now - (float)msecs_tag_time < 500){
        //   // spd_constant = (ros::Time::now().toSec() - tag_time)/0.5; - 이거는 시간을 float으로 하면 안되고, double 로 해야 된다.
        //   // cout << ros::Time::now().toSec() - tag_time << endl;
        //   spd_constant = ((float)msecs_tag_time_now - (float)msecs_tag_time)/500;
        // }

        if(ros::Time::now().toSec() - tag_time < 0.5){
          spd_constant = (ros::Time::now().toSec() - tag_time)/0.5; // - 이거는 시간을 float으로 하면 안되고, double 로 해야 된다.
          spd_constant = spd_constant * spd_constant_tmp;
        }

        float kal_r_vel_tmp = kal_r_vel * spd_constant;
        float kal_f_vel_tmp = kal_f_vel * spd_constant;
        
        if(is_kalman){
          move_right += kal_r_vel_tmp;
          move_front += kal_f_vel_tmp;
          // cout << "kalmanfiltered" << endl;
        }
        // }
        // else{
        //   ROS_ERROR("Tag is lost");
        // }

        float move_up = 0.0;
        
        // 태그보고 고도 조절 - 여기에 태그별로 어떻게 할지 정하는 코드
        // if (tf_name == "tag_6"){
        //   big_tag = false;
        //   // cout << "Tag_6 found" << endl;
        // }
        // if (big_tag){
        //   if(fabs(z_tf - 3.5) > 0.1){
        //     move_up = (3.5 - z_tf) * 0.3;
        //   }
        //   // if(fabs(z_tf - 1.7) > 0.1){
        //   //   move_up = (1.7 - z_tf) * 0.3;
        //   // }
        // }
        // else{
        //   if(fabs(z_tf - 1.3) > 0.1){
        //     move_up = (1.3 - z_tf) * 0.3;
        //   }
        // }
        if(fabs(z_tf - 3.5) > 0.1){
          move_up = (3.5 - z_tf) * 0.3;
        }

        // move_left = -1 * move_right;

        spd_f = move_front;
        spd_r = move_right;
        spd_u = move_up;
        spd_y = 0;
        is_drone_move = true;


        // PID 계수 보내기
        distance_data.data[7] = spd_f;
        distance_data.data[8] = -1 * spd_r;
        distance_data.data[9] = kp_control;
        distance_data.data[10] = kd_control;
        distance_data.data[11] = spd_constant;
        distance_data.data[12] = kal_r_vel_tmp;
        distance_data.data[13] = kal_f_vel_tmp;
        
        distance_pub.publish(distance_data);

        tag_x_tmp = x_tf;
        time_no_tag = ros::Time::now();
      }
      // else{
      //   is_drone_move = false;
      //   ROS_ERROR("Tag is lost");
      // }
      // 여기까지 속도 계산

      // 속도 제어
      if(is_drone_move){
        kp_control = 0; // 직전에는 0.5
        kp_control = 0; // 일단 0으로 해보고 나중에 바꾸기
        kd_control = 0;
        // if(dt < 0.5){
        //   kd_control = 0.01;
        // }

        float yaw = get_yaw(imu_ori_w, imu_ori_x, imu_ori_y, imu_ori_z);
        float drone_vel_f = enu_to_fru_f(drone_vel_e, drone_vel_n, yaw);
        float drone_vel_r = enu_to_fru_r(drone_vel_e, drone_vel_n, yaw);
        
        // PID를 위한 계산
        float proportional_f = spd_f - drone_vel_f;
        float proportional_r = spd_r - drone_vel_r;

        float derivative_f = (proportional_f - (spd_f_prev - drone_vel_f_prev)) / dt;
        float derivative_r = (proportional_r - (spd_r_prev - drone_vel_r_prev)) / dt;

        if(ros::Time::now().toSec() - tag_time < 0.5){
          derivative_f = 0;
          derivative_r = 0;
        }

        // PID 제어
        float input_f = spd_f + proportional_f * kp_control + derivative_f *  kd_control;
        if(fabs(input_f) > max_move_spd){
          input_f = max_move_spd * (input_f)/(fabs(input_f));
        }
        float input_r = spd_r +  proportional_r * kp_control + derivative_r *  kd_control;
        if(fabs(input_r) > max_move_spd){
          input_r = max_move_spd * (input_r)/(fabs(input_r));
        }

        float input_l = -1 * input_r;

        float cont[] = {input_f, input_l, spd_u, spd_y}; // 전좌상회전
        move(cont, n, control_pub);

        // 변수 업데이트
        drone_vel_f_prev = drone_vel_f;
        drone_vel_r_prev = drone_vel_r;
      }
      else{
        float cont[] = {0.0,0.0,0.0,0.0}; // 전좌상회전
        spd_f = 0;
        spd_r = 0;
        move(cont, n, control_pub);
      }
      
      // 1초에 한번씩 배터리 퍼센트 표시하기
      if(ros::Time::now() - battery_time > ros::Duration(1)){
        cout << "Battery : " << (batt_volt/1000) << " V" << endl;
        battery_time = ros::Time::now();
        if(batt_volt/1000 < 14.5){
          float cont[] = {0.0,0.0,0.0,0.0}; //전좌상회전
          move(cont, n, control_pub); // 정지
          // spd_f = 0;
          // spd_r = 0;
          // spd_u = 0;
          // spd_y = 0;
          is_drone_move = false;
          is_run_thread = false;
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
    // gimbal_y = 56;

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

  // auto control_input = []() // 쓰레드 4 드론 Input 제어
  // {
  //   double time_is_control = ros::Time::now().toSec();
  //   float spd_r_prev, spd_f_prev, spd_u_prev, spd_y_prev;
  //   float kp_control = 4.5;
  //   float kd_control = 0.5;
  //   float drone_vel_f_prev = 0;
  //   float drone_vel_r_prev = 0;
  //   int count = 0;

  //   ros::NodeHandle n;
  //   ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  //   while(is_run_thread){
  //     if(is_drone_move){ // 드론 제어
  //       if(spd_r != spd_r_prev || spd_f != spd_f_prev || spd_u != spd_u_prev || spd_y != spd_y_prev){
  //         time_is_control = ros::Time::now().toSec();
  //       }

  //       float dt_control = ros::Time::now().toSec() - time_is_control;

  //       if(dt_control < 0.5){ // 드론의 신호를 받은지 1초가 안되었는지
  //         count += 1;

  //         // 쿼터니언으로 yaw 알아내기
  //         float w = imu_ori_w;
  //         float x = imu_ori_x;
  //         float y = imu_ori_y;
  //         float z = imu_ori_z;

  //         float yaw_x = 2 * (w * z + x * y);
  //         float yaw_y = 1 - 2 * (y * y + z * z);
  //         float yaw = atan2(yaw_x, yaw_y);

  //         // enu to flu
  //         float drone_vel_f = drone_vel_e * cos(yaw) + drone_vel_n * sin(yaw);
  //         float drone_vel_r = drone_vel_e * sin(yaw) - drone_vel_n * cos(yaw);

  //         // PID를 위한 계산
  //         float proportional_f = spd_f - drone_vel_f;
  //         float proportional_r = spd_r - drone_vel_r;

  //         float derivative_f = (proportional_f - (spd_f_prev - drone_vel_f_prev)) / dt_control;
  //         float derivative_r = (proportional_r - (spd_r_prev - drone_vel_r_prev)) / dt_control;

  //         // float d_vel_f = (drone_vel_f - drone_vel_f_prev)/dt_control;
  //         // float d_vel_r = (drone_vel_r - drone_vel_r_prev)/dt_control;
  //         if(count < 2){
  //           // d_vel_f = 0;
  //           // d_vel_r = 0;
  //           derivative_f = 0;
  //           derivative_r = 0;
  //         }

  //         // // 임시
  //         // derivative_f = 0;
  //         // derivative_u = 0;

  //         // PID를 이용한 제어
  //         float input_f = spd_f + proportional_f * kp_control + derivative_f *  kd_control;
  //         // input_f = spd_f + input_f;
  //         if(input_f > max_move_spd){
  //           input_f = max_move_spd;
  //         }
  //         else if(input_f < -1 * max_move_spd){
  //           input_f = -1 * max_move_spd;
  //         }

  //         float input_r = spd_r +  proportional_r * kp_control + derivative_r *  kd_control;
  //         if(input_r > max_move_spd){
  //           input_r = max_move_spd;
  //         }
  //         else if(input_r < -1 * max_move_spd){
  //           input_r = -1 * max_move_spd;
  //         }
  //         float input_l = -1 * input_r;
          
  //         float cont[] = {input_f, input_l, spd_u, spd_y}; // 전좌상회전
  //         move(cont, n, control_pub);


  //         // 변수 업데이트
  //         drone_vel_f_prev = drone_vel_f;
  //         drone_vel_r_prev = drone_vel_r;
  //         // 조금 기다리기
  //         while(ros::Time::now().toSec() - time_is_control < 0.02){

  //         }
  //       }
  //       else{
  //         is_drone_move = false;
  //         count = 0;
  //         float cont[] = {0.0,0.0,0.0,0.0};
  //         move(cont, n, control_pub);
  //       }
  //     }
  //     else{ // 드론 제어 안함.
  //       float cont[] = {0.0,0.0,0.0,0.0};
  //       count = 0;
  //       move(cont, n, control_pub);
  //     }
  //   }
  // };

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  std::thread t3 = std::thread(control_gimbal);
  // std::thread t4 = std::thread(control_input);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  t3.join();
  // t4.join();

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
  if (is_tag_signal){
    tf_name = msg3.transforms[0].child_frame_id;
    if (tf_name == "tag_2"){
      // cout << "true" << endl;
      count_tf = 0;
      big_tag = false;
    }
    else{
      // cout << tf_name << endl;
      count_tf += 1;
      // if (count_tf > 1){
      //   cout << count_tf << endl;
      // }
      big_tag = true;
    }
    cout << tf_name << endl;

    // 이전 코드
    x_tf = msg3.transforms[0].transform.translation.x;
    y_tf = msg3.transforms[0].transform.translation.y;
    z_tf = msg3.transforms[0].transform.translation.z;
    // // 태그 크기 확인용 임시 코드
    // cout << "Distance : " << z_tf << endl;
    is_tag_signal = true;
    // cout << "x_tf : " << x_tf << endl;
    // cout << "y_tf : " << y_tf << endl;
    // cout << "z_tf : " << z_tf << endl;

    // // 20221030추가
    // // string tf_name_tmp = msg3.transforms[0].child_frame_id;
    // // tf_name = tf_name_tmp;
    // tf_name = msg3.transforms[0].child_frame_id;
    // if (tf_name == "tag_6"){
    //   cout << "True" << endl;
    //   count_tf = 0;
    // }
    // else{
    //   cout << tf_name << endl;
    //   count_tf += 1;
    //   if (count_tf > 1){
    //     cout << count_tf << endl;
    //   }
    // }
    // // cout << msg3.transforms[0] << endl;
    // // cout << msg3.transforms[0].child_frame_id << endl;

    // // tf_name = msg3.child_frame_id[0];
  }
}

void callback_tf_2(const geometry_msgs::Transform tag_tmp){
  if (is_tag_signal == false){
    // cout << tag_tmp << endl;
    x_tf = tag_tmp.translation.x;
    // cout << x_tf << endl;
    y_tf = tag_tmp.translation.y;
    z_tf = tag_tmp.translation.z;
    // cout << "tf_2" << endl;

  }
}

void cb_gps_velocity(const geometry_msgs::Vector3Stamped vel){
    drone_vel_e = vel.vector.x;
    drone_vel_n = vel.vector.y;
    // drone_vel_u = vel.vector.z;
}

float get_yaw(float w, float x, float y, float z){
  float yaw_x = 2 * (w * z + x * y);
  float yaw_y = 1 - 2 * (y * y + z * z);
  float yaw = atan2(yaw_x, yaw_y);

  return yaw;
}

float enu_to_fru_f(float e, float n, float y){
  float f = e * cos(y) + n * sin(y);
  return f;
}

float enu_to_fru_r(float e, float n, float y){
  float r = e * sin(y) - n * cos(y);
  return r;
}