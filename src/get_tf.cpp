#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
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

// 코드 정보
// 코드 이름 : get_tf
// Tag 정보를 화면에 출력해주기만 한다.
// !! (느낌표 두개)있는거 ctrl + f 해서 이 부분들 수정하기

// // 전역 변수 정의
bool is_start = false;
using namespace std;
// 짐벌 제어
float gimbal_x, gimbal_y;
// Ctrl+C일때 바로 종료하기 위함.
bool is_run_thread;
// 태그 위치
float x_tf, y_tf, z_tf;
// double time_prev;
float gimbal_down;
bool is_gimbal_down = true; // 밖에서 할때는 false, 안에서 할때는 true

// // 함수 정의
void callback_tf(const tf2_msgs::TFMessage msg3); // Tag 정보 - 거리 기반 PID

std_msgs::Float64MultiArray gim;

int main(int argc, char **argv){
  if(is_gimbal_down){
    gimbal_down = 76;
  }
  else{
    gimbal_down = 56;
  }
  
  is_run_thread = true;
  ros::init(argc, argv, "get_tf");
  ros::NodeHandle n;

  // // subscriber - 영상 정보 및 얻을 수 있는 센서 데이터 전부 받기
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
  
  // // publisher
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);

  
  // 짐벌 초기화
  gim.data.push_back(50); // 이거 해야지 크기 2인 배열 만들 수 있는 듯
  gim.data.push_back(0);
  // gim.data[0] = 56; // 아래 바라보기 - 76이면 정면
  // gim.data[0] = 76; // 아래 바라보기 - 76이면 정면 - 나가서 할때는 주석
  gim.data[0] = gimbal_down;
  gim.data[1] = 77; // 좌우 77이면 정면
  gimbal_control.publish(gim); // 짐벌 초기화

  // // // // 쓰레드 1 - Get sensor data
  auto ros_spin = []()
	{
    if(is_run_thread){
      cout << "start ros spin" << endl;
      ros::spin(); // 이거 굳이 쓰레드 한개를 차지할 필요가 있나? 그냥 메인함수에 남겨둬도 될거 같은데.
    }
    is_run_thread = false; // 이거 오류 나는지 확인 !!
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
  std::thread t3 = std::thread(control_gimbal);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
  t3.join();

  return 0;
}

void callback_tf(const tf2_msgs::TFMessage msg3) // need to be edited
{ // 값 저장하는 코드
  x_tf = msg3.transforms[0].transform.translation.x;
  y_tf = msg3.transforms[0].transform.translation.y;
  z_tf = msg3.transforms[0].transform.translation.z;
  cout << "x_tf : " << x_tf << endl;
  cout << "y_tf : " << y_tf << endl;
  cout << "z_tf : " << z_tf << endl;
  cout << endl;
}