#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dji_sdk/DroneArmControl.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
#include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름
#include "geometry_msgs/Vector3.h"
// #include <pluginlib/class_list_macros.h>
// #include "apriltag_ros/common_functions.h"
#include "tf2_msgs/TFMessage.h"

#include <sstream>

uint32_t a; // timestamp 시뮬레이션이랑 맞추기 위함
uint32_t b;
std::string f_id;

float tf_x;

ros::ServiceClient sdk_ctrl_authority_service; // 드론 제어 권한 관련
ros::ServiceClient drone_task_service;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

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


void cb(const sensor_msgs::Imu msg)
{
  std::cout << msg.header.stamp << std::endl;
  
  a = msg.header.stamp.sec;
  b = msg.header.stamp.nsec;
  f_id = msg.header.frame_id;
  
  ROS_INFO("%d",a);
  std::cout << a << b << std::endl;
}

void move(float cont[], float dur);
void spd(float a[],float b, float c, float d, float e){ // 파이썬 처럼 배열 변경하기 위해 만든 함수.
  a[0] = b;
  a[1] = c;
  a[2] = d;
  a[3] = e;
}

float reset_array(float cont[]){
  for (int i = 0;i < 4; i++){
    cont[i] = 0.0;
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker_ros");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
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

  ROS_INFO("Wait 2s");
  ros::Duration(2).sleep();
  bool takeoff_result; // ** 주의 ** 이륙 관련 코드
  
  /*
  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
    ROS_ERROR("**Caution** If you see this error you must edit this code before real flight!");
    takeoff_result = true; // 나중에 문제가 될 수도 있으니 주의
  }
  else {
    takeoff_result = monitoredTakeoff(); // 이륙 했는지 확인
  }
  */
  
  // temp(for test 무슨 센서 데이터가 출력이 안되는지 확인용)
  std::cout << "Before take off" << std::endl;
  std::cout << "Status on ground : " <<DJISDK::FlightStatus::STATUS_ON_GROUND << std::endl;
  std::cout << "Mode engine start : "<<DJISDK::DisplayMode::MODE_ENGINE_START << std::endl;
  
  takeoff_result = monitoredTakeoff();
  std::cout << takeoff_result << std::endl;
  
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Subscriber sss = n.subscribe("dji_sdk/imu", 1000, cb);
  
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_ros", 1000); // 예제
  //ros::Publisher control_gen_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  //ros::Publisher control_pos_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ros::Publisher control_vel_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  //ros::Publisher control_rate_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  
  // subscribe tf
  // ros::Subscriber tf_sub = n.subscribe("dji_sdk/tf", 1000, cb);


  ros::Rate loop_rate(50);
  
  int count = 1;
  //float input_drone = 20.0;
  //float chk = 20.0;
  while (ros::ok())
  {
    if(takeoff_result)
    {
      
      if(ros::ok()){
      
        sensor_msgs::Joy msg_control_gen;
        sensor_msgs::Joy msg_control_vel;
        //obtain_control();
        
        /*
        uint8_t flag = (DJISDK::VERTICAL_VELOCITY   | // when using generic
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_BODY  |
                    DJISDK::STABLE_ENABLE);      
        //msg_control.axes = {0.0, 70.0,0.0,0.0}; // 이거만 있으면 드론 움직임.  , {0.0,80.0,0.0,0.0}
        msg_control_gen.axes.push_back(0.0);
        msg_control_gen.axes.push_back(90.0);
        msg_control_gen.axes.push_back(0.0);
        msg_control_gen.axes.push_back(0.0);
        msg_control_gen.axes.push_back(flag);
        */
        
        msg_control_vel.axes.push_back(0.0);
        msg_control_vel.axes.push_back(50.0);
        msg_control_vel.axes.push_back(0.0);
        msg_control_vel.axes.push_back(0.0);
        
        
        msg_control_vel.header.seq = count;
        //msg_control_vel.header.stamp = ros::Time::now();
        msg_control_vel.header.stamp.sec = a;
        msg_control_vel.header.stamp.nsec = b;
        msg_control_vel.header.frame_id = f_id;
        std::cout << msg_control_vel << std::endl;
        ros::Duration(0.02).sleep();
        //input_drone = input_drone + 5.0;
        //control_gen_pub.publish(msg_control_gen);
        control_vel_pub.publish(msg_control_vel);
        
        /* demo flight control example
        sensor_msgs::Joy controlPosYaw;

        controlPosYaw.axes.push_back(xCmd);
        controlPosYaw.axes.push_back(yCmd);
        controlPosYaw.axes.push_back(zCmd);
        controlPosYaw.axes.push_back(yawDesiredRad);
        ctrlPosYawPub.publish(controlPosYaw);
        */
        
        //ROS_INFO("F : %f", msg_control.axes[0]);
        //ROS_INFO("R : %f", msg_control.axes[1]); 
        //ROS_INFO("U : %f", msg_control.axes[2]); 
        //ROS_INFO("Y : %f", msg_control.axes[3]); 
        //ROS_INFO(" ");
        //ros::Duration(0.01).sleep(); // 단위는 초(?)
      }
      else{
        //input_drone = 10.0;
      }
    }
    else{
      ROS_ERROR("takeoff fail!");
    }
    
    ros::spinOnce(); // 이거 뭐지?
    //ros::spin();
    loop_rate.sleep();
    ++count;
    ROS_INFO("%d",count);
  }

  return 0;
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

