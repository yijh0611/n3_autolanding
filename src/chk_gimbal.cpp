// /// control gimbal with apriltag ///
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf2_msgs/TFMessage.h"
// multi thread
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <string>

#include<unistd.h>
#include<termios.h>

#include<math.h>

using namespace std;

int angle_gimbal; // 각도 전역변수
float x_tf;
float y_tf;
float z_tf;
float gimbal_x;
float gimbal_y;
float chk_x;
float chk_y;
double ang_x;
double ang_y;
bool is_run_thread;

int getch();

void callback_tf(const tf2_msgs::TFMessage msg);
std_msgs::Float64MultiArray gim;


int main(int argc, char **argv){

  cout << "Start test_code" << endl;
  ROS_INFO("Start test_code");
  is_run_thread = true;

  // // 역삼각함수 확인
  // double x,ret;

  // x = 1;
  // ret = atan(x); 
  // printf("atan( %lf ) = %lf\n", x, ret);
  // cout << M_PI << endl;
  // ret = ret * 180/M_PI;
  // cout << ret << endl;

  // return 0;
  // // 삼각함수 확인 여기까지

  ros::init(argc, argv, "test");
  ros::NodeHandle n;
    
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  std_msgs::Float64MultiArray gim;

  // 여기서부터 정보 받고, 제어하는 코드
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf); // tf 값 수신

  auto ros_spin = []()
	{
    cout << "Start ros spin" << endl;
		ros::spin();
    is_run_thread = false;
	};

  // 쓰레드 2 - Control gimbal
  auto control_drone = []()
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
    std_msgs::Float64MultiArray gim; // 위에 있어서 일단 주석처리 했음. 안되면 주석 풀기
    gim.data.push_back(77); // 상하 // 위에서 이미 해서 지웠음.
    gim.data.push_back(65); // 좌우
    gimbal_control.publish(gim);
    ros::Duration(1).sleep();

    gimbal_x = 76;
    gimbal_y = 77;
    x_tf = 0;
    y_tf = 0;
    chk_x = 0;
    chk_y = 0;
    gim.data[0] = gimbal_y;
    gim.data[1] = gimbal_x;
    gimbal_control.publish(gim);
    ros::Duration(1).sleep();

    while(is_run_thread){
      // gimbal x 방향 회전 (pan // 65~87) 각도 단위는 8도
      if(chk_x != x_tf){

        // if(x_tf > 0.05){// 이거는 확인해보기
        //   gimbal_x = gimbal_x + 1;
        // }
        // else if(x_tf < -0.05){
        //   gimbal_x = gimbal_x - 1;
        // }
        int x_threshold = 16; // 8
        ang_x = atan(x_tf/z_tf)*180/M_PI;
        if (abs(ang_x) > x_threshold - 1){
          gimbal_x = gimbal_x + ang_x/x_threshold;
        }
        
        if(gimbal_x > 87){
          gimbal_x = 87;
        }
        else if(gimbal_x < 65){
          gimbal_x = 65;
        }      

        chk_x = x_tf;
      }

      // gimbal y 방향 회전 (tilt // 56 ~ 77(77이 정면이고, 더 안올라감)) 각도 단위는 4.286도
      if(chk_y != y_tf){
        // if(y_tf > 0.05){// 이거는 확인해보기
        //   gimbal_y = gimbal_y - 1;
        // }
        // else if(y_tf < -0.05){
        //   gimbal_y = gimbal_y + 1;
        // }
        int y_threshold = 8; // 4
        ang_y = atan(y_tf/z_tf)*180/M_PI;
        if(abs(ang_y) > y_threshold - 1){
          gimbal_y = gimbal_y - ang_y / y_threshold;
        }

        if(gimbal_y > 79){
            gimbal_y = 79;
        }
        else if(gimbal_y < 56){
          gimbal_y = 56;
        }
        chk_y = y_tf;
      }

      gim.data[1] = gimbal_x;
      gim.data[0] = gimbal_y;

      // cout << gim << endl;
      gimbal_control.publish(gim); // 값이 바뀌는것과 상관 없이 같은 값 publish
      
      ros::Duration(0.3).sleep(); // sleep for hundredth of a second
    }
	};

  // 쓰레드 3 - Get button input
  auto get_button = [](){
    cout << "start get button" << endl;
    while(is_run_thread){
      int c1 = getch();    // 입력 버퍼를 사용하지 않음, 화면에 키의 입력을 보여주지 않음
      cout << "Key input : " << c1 << endl;
      if(c1 == 99){ // "C 입력시 종료"
        is_run_thread = false;
      }
    }
  };

  std::thread t1 = std::thread(ros_spin);
	std::thread t2 = std::thread(control_drone);
  std::thread t3 = std::thread(get_button);
  t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
	t2.join();
  // t3.join();
  
  return 0;
}

void callback_tf(const tf2_msgs::TFMessage msg) // need to be edited
{
  x_tf = msg.transforms[0].transform.translation.x;
  y_tf = msg.transforms[0].transform.translation.y;
  z_tf = msg.transforms[0].transform.translation.z;
}

int getch()
{
  int c;
  struct termios oldattr, newattr;

  tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
  newattr = oldattr;
  newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
  newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
  newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
  tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
  c = getchar();                               // 키보드 입력 읽음
  tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
  return c;
}