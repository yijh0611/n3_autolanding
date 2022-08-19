#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "dji_sdk/DroneArmControl.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
#include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름
#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"

#include <sstream>
// 이미지 출력을 위한 임시 코드
#include "sensor_msgs/CameraInfo.h"

// 이륙하는 부분 제거 했음.

uint32_t a; // timestamp 시뮬레이션이랑 맞추기 위함
uint32_t b;
std::string f_id;

ros::ServiceClient sdk_ctrl_authority_service; // 드론 제어 권한 관련
ros::ServiceClient drone_task_service;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

// 함수 정의
void chatterCallback2(const sensor_msgs::Imu msg2);
void cb(const sensor_msgs::Imu msg);
void spd(float a[],float b, float c, float d, float e);// 파이썬 처럼 배열 변경하기 위해 만든 함수.
void move(float cont[], float dur);
float reset_array(float cont[]);
void follow_tag();
void callback_tf(const tf2_msgs::TFMessage msg3);

int main(int argc, char **argv){

  ros::init(argc, argv, "talker_ros");
  ros::NodeHandle n;
  // control하기위해 추가된 부분

  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  //ctrlBrakePub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = n.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control"); // 이거 없으면 이륙했는지 확인 불가
  
  ROS_INFO("Wait 2s");
  ros::Duration(2).sleep();
  bool obtain_control_result = obtain_control(); //비행 권한 얻는 코드
    
  ros::Subscriber sss = n.subscribe("dji_sdk/imu", 1000, cb);
  ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf); // tf 값 수신
  //ros::Publisher control_gen_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  
  // float gim[2] = {30,50}; // 이거는 안쓸 거임.
  std_msgs::Float64MultiArray gim;
  gim.data[0] = 30;
  gim.data[1] = 50;
  gimbal_control.publish(gim);


  /*
  ros::Publisher control_gimbal = n.advertise<dji_sdk::Gimbal>("dji_sdk/gimbal_angle_cmd",10);
  
  dji_sdk::Gimbal gimbal_control;
  
  std::cout << gimbal_control << std::endl;
  //gimbal_control.x = 90.0;
  //gimbal_control.y = 90.0;
  //gimbal_control.z = 90.0;
  std::cout << gimbal_control << std::endl;
    
  ros::Duration(0.02).sleep();
  control_gimbal.publish(gimbal_control);
  */
    
  /////////// 아래 코드 대신에 while문 넣으면, 계속 최신 값 수신 가능
  ros::Rate loop_rate(50);
  
  // 이동하는 코드
  int count = 1;
  ros::Time start_time = ros::Time::now();

  float cont[] = {1.0,0.0,0.0,0.0}; // front right up yaw
  // move(cont,0.5);

  spd(cont,0.0,0.0,0.0,2.0);
  move(cont,2.0);

  spd(cont,0.0,0.0,0.0,0.0);
  move(cont,1.0);

  spd(cont,0.0,0.0,0.0,-2.0);
  move(cont,2.0);

  spd(cont,0.0,0.0,0.0,0.0);
  move(cont,1.0);
  
  spd(cont,0.0,0.0,1.0,0.0);
  move(cont,1.5);

  return 0;
}



void follow_tag(){
  /*
  1. 태그 tf정보 받아서 분석
  2. 태그 정보 활용해서 태그 따라가기
  3. tf 정보 없을 경우 오류 메세지를 표시하고 드론 중지
  4. 완료시 드론 착륙하고, 이 단계에서 잘못되는 경우 앞 단계로 이동하거나 다른 조치를 취하도록 만들기
  */
  while(true){
    // 1. 태그 tf정보 받아서 분석
    break;


  }
}

float reset_array(float cont[]){
  for (int i = 0;i < 4; i++){
    cont[i] = 0.0;
  }
}

void cb(const sensor_msgs::Imu msg)
{
  std::cout << msg.header.stamp << std::endl;
  
  a = msg.header.stamp.sec;
  b = msg.header.stamp.nsec;
  f_id = msg.header.frame_id;
  
  ROS_INFO("%d",a);
  std::cout << a << b << std::endl;
}

void spd(float a[],float b, float c, float d, float e){ // 파이썬 처럼 배열 변경하기 위해 만든 함수.
  a[0] = b;
  a[1] = c;
  a[2] = d;
  a[3] = e;
}

void chatterCallback2(const sensor_msgs::Imu msg2)
{
  ROS_INFO("Orientation x: [%f]", msg2.orientation.x);
  ROS_INFO("Orientation y: [%f]", msg2.orientation.y);
  ROS_INFO("Orientation z: [%f]", msg2.orientation.z);

  ROS_INFO("Angular velocity x: [%f]", msg2.angular_velocity.x);
  ROS_INFO("Angular velocity y: [%f]", msg2.angular_velocity.y);
  ROS_INFO("Angular velocity z: [%f]", msg2.angular_velocity.z);

  ROS_INFO("Linear Acc x: [%f]", msg2.linear_acceleration.x);
  ROS_INFO("Linear Acc y: [%f]", msg2.linear_acceleration.y);
  ROS_INFO("Linear Acc z: [%f]", msg2.linear_acceleration.z);
}

void callback_tf(const tf2_msgs::TFMessage msg3)
{
  std::cout << msg3.transforms[0] << std::endl;
  std::cout << "" << std::endl;
  std::cout << msg3.transforms[0].transform.translation.x << std::endl;
  std::cout << "" << std::endl;
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
  
  //ROS_INFO("Wait 2s");
  //ros::Duration(2).sleep();
  return true;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    ROS_ERROR("We will ignore this problem");
    return false;
  }

  return true;
}

bool monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();
    
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    ROS_ERROR("Drone is not on ground!");
    ROS_ERROR("But we will ignore this problem"); //위에서 함수를 한번 호출했기 때문에 문제가 생겼을 수도 있다.
    //return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(1)) {
    ros::Duration(0.01).sleep();
    //ROS_INFO("Waiting...");
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(1)) {
    //ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    ROS_ERROR("Skip motor check."); // ERROR
    //ROS_ERROR("But we will ignore this problem.");
    //ROS_ERROR("Maybe it is because the sdk cannot get the state of this drone.");
    //ROS_ERROR("Return false.");
    //std::cout << DJISDK::FlightStatus::STATUS_ON_GROUND << std::endl;
    //std::cout << DJISDK::DisplayMode::MODE_ENGINE_START << std::endl;
    //return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(1)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration()) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning."); // error
    ROS_ERROR("Skip flying check"); // ERROR
    
    ROS_INFO("Wait 7s");
    ros::Duration(7).sleep();
    //ROS_ERROR("But we will ignore this problem.");
    //ROS_ERROR("Maybe it is because the sdk cannot get the state of this drone.");
    //return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(10)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

void move(float control[],float dur){
  ros::Time start_time = ros::Time::now();
  ros::Rate loop_rate(50);
  ros::NodeHandle n;
  //ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10); // 이거로 해야지 NEU가 아니라 FRU로 할 수 있다.
  
  while (true){
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
        
    ROS_INFO("F : %f", msg_control.axes[0]);
    ROS_INFO("R : %f", msg_control.axes[1]); 
    ROS_INFO("U : %f", msg_control.axes[2]); 
    ROS_INFO("Y : %f", msg_control.axes[3]); 
    ROS_INFO(" ");
    
    ros::spinOnce();
    loop_rate.sleep();
    
    if(ros::Time::now() - start_time > ros::Duration(dur)) {
      break;
    }
  }
}

/*
int cb(const sensor_msgs::Imu msg)
{
  std::cout << msg.header.stamp << std::endl;
  int a = msg.header.stamp.sec;
  int b = msg.header.stamp.nsec;
  
  std::cout << a << b << std::endl;
  
  return a,b;
}
*/

/*
void cb(const sensor_msgs::Imu msg)
{
  //ROS_INFO("test [%d]", msg.header.stamp.sec);
  
  std::cout << msg.header.stamp << std::endl;
  int a = msg.header.stamp.sec;
  int b = msg.header.stamp.nsec;
  
  ROS_INFO("%d",a);
  std::cout << a << b << std::endl;
  
  sensor_msgs::Joy msg_control_vel;
        
  msg_control_vel.axes.push_back(0.0);
  msg_control_vel.axes.push_back(50.0);
  msg_control_vel.axes.push_back(0.0);
  msg_control_vel.axes.push_back(0.0);        
        
  std::cout << msg_control_vel << std::endl;
        //ros::Duration(0.02).sleep();
        
  control_vel_pub.publish(msg_control_vel);
}*/

