#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dji_sdk/DroneArmControl.h"
#include "sensor_msgs/Joy.h"
#include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
#include "dji_sdk/demo_flight_control.h" // 이게 있어야 dji_sdk::SDKControlAuthority가 되는 듯 // dji_sdk/demo...에서 dji_sdk는 헤더 파일이 있는 폴더 이름

#include <sstream>

ros::ServiceClient sdk_ctrl_authority_service; // 드론 제어 권한 관련
ros::ServiceClient drone_task_service;

// ros::Publisher ctrlPosYawPub; //이거는 없어도 되는 듯?
// ros::Publisher ctrlBrakePub; //이거는 없어도 되는 듯?


// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

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
  // Publish the control signal
  //ctrlPosYawPub = n.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  //ctrlBrakePub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = n.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control"); // 이거 없으면 이륙했는지 확인 불가
  
  bool obtain_control_result = obtain_control(); //비행 권한 얻는 코드
  bool takeoff_result; // ** 주의 **
  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
    ROS_ERROR("**Caution** If you see this error you must edit this code before real flight!");
    takeoff_result = true; // 나중에 문제가 될 수도 있으니 주의 
  }
  else {
    takeoff_result = monitoredTakeoff(); // 이륙 했는지 확인
  }
  
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
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_ros", 1000); // 예제
  ros::Publisher control_pub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10); //rollpitch_yawrate_zposition

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   
  
  int count = 0;
  float input_drone = 10.0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    /*
    std_msgs::String msg;

    std::stringstream ss;
    if(0){
      ss << "hello world ros " << count ; // << DJI::OSDK::Version::M100_31; // << DJI::OSDK::Status;
    }
    else{
      ss << "hello world ros " << count;
    }
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    */

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);
    
    if(takeoff_result)
    { // 여기에 코드 추가
      if(input_drone < 100){
        sensor_msgs::Joy msg_control;
        msg_control.axes = {0.0,input_drone,10.0,0.0};
        input_drone = input_drone + 5.0;
        control_pub.publish(msg_control);
        ROS_INFO("F : %f", msg_control.axes[0]); //axes[1]
        ROS_INFO("R : %f", msg_control.axes[1]); 
        ROS_INFO("U : %f", msg_control.axes[2]); 
        ROS_INFO("Y : %f", msg_control.axes[3]); 
        ROS_INFO(" ");
        ros::Duration(0.5).sleep();
      //obtain_control(); // 다시 권한을 얻어야 명령이 계속 들어가나?
      }
      else{
        input_drone = 10.0;
      }
    }
    else{
      ROS_ERROR("takeoff fail!");
    }
    
    //ros::spinOnce(); // 이거 뭐지?
    loop_rate.sleep();
    ++count;
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
  
  ROS_INFO("Wait 2s");
  ros::Duration(2).sleep();
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
         ros::Time::now() - start_time < ros::Duration(10)) {
    ros::Duration(0.01).sleep();
    //ROS_INFO("Waiting...");
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(10)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    ROS_ERROR("But we will ignore this problem.");
    ROS_ERROR("Maybe it is because the sdk cannot get the state of this drone.");
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
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    //ROS_INFO("Waiting...");
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    ROS_ERROR("But we will ignore this problem.");
    ROS_ERROR("Maybe it is because the sdk cannot get the state of this drone.");
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
