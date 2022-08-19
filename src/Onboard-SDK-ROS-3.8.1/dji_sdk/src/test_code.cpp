// // // // // // // // #include "ros/ros.h"
// // // // // // // // #include "std_msgs/String.h"
// // // // // // // // #include "sensor_msgs/Imu.h"
// // // // // // // // #include <stdint.h>
// // // // // // // // #include <string.h>
// // // // // // // // #include <stdlib.h>
// // // // // // // // #include "tf2_msgs/TFMessage.h"
// // // // // // // // // #include "geometry_msgs/TransformStamped.h"

// // // // // // // // void chatterCallback(const std_msgs::String::ConstPtr& msg)
// // // // // // // // {
// // // // // // // //   ROS_INFO("I heard: [%s]", msg->data.c_str());
// // // // // // // // }

// // // // // // // // float tf_x;

// // // // // // // // // void callback_tf(const tf2_msgs::TFMessage msg3)
// // // // // // // // // void callback_tf(const geometry_msgs::TransformStamped msg3)
// // // // // // // // void callback_tf(const tf2_msgs::TFMessage msg3)
// // // // // // // // {
// // // // // // // //   std::cout << msg3.transforms[0] << std::endl;
// // // // // // // //   std::cout << "" << std::endl;
// // // // // // // //   std::cout << msg3.transforms[0].transform.translation.x << std::endl;
// // // // // // // //   std::cout << "" << std::endl;

// // // // // // // //   tf_x = msg3.transforms[0].transform.translation.x;
// // // // // // // //   std::cout << tf_x << std::endl;
// // // // // // // //   std::cout << "" << std::endl;
// // // // // // // // }

// // // // // // // // void chatterCallback2(const sensor_msgs::Imu msg2)
// // // // // // // // {
// // // // // // // //   ROS_INFO("Orientation x: [%f]", msg2.orientation.x);
// // // // // // // //   ROS_INFO("Orientation y: [%f]", msg2.orientation.y);
// // // // // // // //   ROS_INFO("Orientation z: [%f]", msg2.orientation.z);

// // // // // // // //   ROS_INFO("Angular velocity x: [%f]", msg2.angular_velocity.x);
// // // // // // // //   ROS_INFO("Angular velocity y: [%f]", msg2.angular_velocity.y);
// // // // // // // //   ROS_INFO("Angular velocity z: [%f]", msg2.angular_velocity.z);

// // // // // // // //   ROS_INFO("Linear Acc x: [%f]", msg2.linear_acceleration.x);
// // // // // // // //   ROS_INFO("Linear Acc y: [%f]", msg2.linear_acceleration.y);
// // // // // // // //   ROS_INFO("Linear Acc z: [%f]", msg2.linear_acceleration.z);
// // // // // // // // }

// // // // // // // // /*
// // // // // // // // void get_vel(const geometry_msgs::Vector3Stamped vel)
// // // // // // // // {
// // // // // // // //   ROS_INFO("Velocity: [%f]", vel[1]);
// // // // // // // //   //ROS_INFO("Velocity: [%f]", vel.vector[1]);
// // // // // // // //   //ROS_INFO("Velocity: [%f]", vel.vector[2]);
// // // // // // // // }
// // // // // // // // */

// // // // // // // // int main(int argc, char **argv)
// // // // // // // // {
// // // // // // // //   ros::init(argc, argv, "listener_ros");
// // // // // // // //   ros::NodeHandle n; //노드 핸들은 한개만 있어도 여러개 subscribe가 가능한 것 같다.
  
// // // // // // // //   //ros::Subscriber sub = n.subscribe("chatter_ros", 1000, chatterCallback);
// // // // // // // //   // ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, chatterCallback2);
// // // // // // // //   ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf);
// // // // // // // // //  ros::Subscriber sub3 = n3.subscribe("dji_sdk/angular_velocity_fused", 1000, get_vel); //n3 대신에 n으로 해도 되는거 같다. - 확인 필요
// // // // // // // // /**  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback_practice);
// // // // // // // // **/

// // // // // // // //   ros::spin();

// // // // // // // //   return 0;
// // // // // // // // }


// // // // // // // // 짐벌 제어 테스트
// // // // // // // #include "ros/ros.h"
// // // // // // // #include "std_msgs/Float64MultiArray.h"
// // // // // // // #include "dji_sdk/dji_sdk.h" // 이거 하려면 cmake에서 디렉토리 설정해줘야 함.
// // // // // // // #include <iostream>

// // // // // // // using namespace std;

// // // // // // // int main(int argc, char **argv){

// // // // // // //   ros::init(argc, argv, "test_code");
// // // // // // //   ros::NodeHandle n;
  
  
// // // // // // //   ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
  
// // // // // // //   while(1){
// // // // // // //     cout << 1 << endl;
// // // // // // //     std_msgs::Float64MultiArray gim;
// // // // // // //     gim.data = {500,1000};
// // // // // // //     gimbal_control.publish(gim);
// // // // // // //     cout << 2 << endl;
// // // // // // //     ROS_INFO("Wait 6s");
// // // // // // //     ros::Duration(6).sleep();
// // // // // // //   }
  

// // // // // // //   /////////// 아래 코드 대신에 while문 넣으면, 계속 최신 값 수신 가능
// // // // // // //   ros::Rate loop_rate(50);
  
// // // // // // //   return 0;
// // // // // // // }




// // // // // // // // landing test
// // // // // // // // ******
// // // // // // // #include "dji_sdk/demo_flight_control.h"
// // // // // // // #include "dji_sdk/dji_sdk.h"
// // // // // // // #include <iostream>

// // // // // // // const float deg2rad = C_PI/180.0;
// // // // // // // const float rad2deg = 180.0/C_PI;

// // // // // // // ros::ServiceClient sdk_ctrl_authority_service;
// // // // // // // ros::ServiceClient drone_task_service;
// // // // // // // ros::ServiceClient query_version_service;

// // // // // // // ros::Publisher ctrlPosYawPub;
// // // // // // // ros::Publisher ctrlBrakePub;

// // // // // // // // global variables for subscribed topics
// // // // // // // uint8_t flight_status = 255;
// // // // // // // uint8_t display_mode  = 255;
// // // // // // // sensor_msgs::NavSatFix current_gps;
// // // // // // // geometry_msgs::Quaternion current_atti;

// // // // // // // Mission square_mission;
// // // // // // // //void Mission::step(); // dhfb qkdwldyd; gkatn wjddml

// // // // // // // int main(int argc, char** argv)
// // // // // // // {
// // // // // // //   ROS_INFO("Test landing");
// // // // // // //   ros::init(argc, argv, "demo_flight_control_node");
// // // // // // //   ros::NodeHandle nh;

// // // // // // //   // Subscribe to messages from dji_sdk_node
// // // // // // //   ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
// // // // // // //   ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
// // // // // // //   ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
// // // // // // //   ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);

// // // // // // //   // Publish the control signal
// // // // // // //   ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
// // // // // // //   // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
// // // // // // //   // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
// // // // // // //   // properly in function Mission::step()
// // // // // // //   ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
// // // // // // //   // Basic services
// // // // // // //   sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
// // // // // // //   drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
// // // // // // //   query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

// // // // // // //   bool obtain_control_result = obtain_control();
// // // // // // //   bool takeoff_result;
// // // // // // //   if(is_M100())
// // // // // // //   {
// // // // // // //     ROS_INFO("M100 taking off!");
// // // // // // //     takeoff_result = M100monitoredTakeoff();
// // // // // // //   }
// // // // // // //   else
// // // // // // //   {
// // // // // // //     ROS_INFO("A3/N3 taking off!");
// // // // // // //     takeoff_result = monitoredTakeoff();
// // // // // // //   }
// // // // // // //   ROS_INFO("Test ros spin loop");
// // // // // // //   if(takeoff_result)
// // // // // // //   {
// // // // // // //     square_mission.reset();
// // // // // // //     square_mission.start_gps_location = current_gps;
// // // // // // //     //ROS_INFO(current_gps);
// // // // // // //     //std::cout << current_gps << std::endl;
// // // // // // //     square_mission.setTarget(0, 20, 3, 60);
// // // // // // //     square_mission.state = 1;
// // // // // // //     ROS_INFO("##### Start route %d ....", square_mission.state);
// // // // // // //     //ROS_ERROR("##### Start route %d ....", square_mission.state); // this is the end of the code
// // // // // // //     //step();
    
// // // // // // //   }
// // // // // // //   //void start_demo() = Mission::step();
  
// // // // // // //   //ROS_INFO("End of the code");

// // // // // // //   ros::spin();
// // // // // // //   //ROS_INFO("Return 0");
// // // // // // //   return 0;
// // // // // // // }

// // // // // // // // Helper Functions

// // // // // // // /*! Very simple calculation of local NED offset between two pairs of GPS
// // // // // // // /coordinates. Accurate when distances are small.
// // // // // // // !*/
// // // // // // // void
// // // // // // // localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
// // // // // // //                          sensor_msgs::NavSatFix& target,
// // // // // // //                          sensor_msgs::NavSatFix& origin)
// // // // // // // {
// // // // // // //   double deltaLon = target.longitude - origin.longitude;
// // // // // // //   double deltaLat = target.latitude - origin.latitude;

// // // // // // //   deltaNed.y = deltaLat * deg2rad * C_EARTH;
// // // // // // //   deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
// // // // // // //   deltaNed.z = target.altitude - origin.altitude;
// // // // // // // }


// // // // // // // geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
// // // // // // // {
// // // // // // //   geometry_msgs::Vector3 ans;

// // // // // // //   tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
// // // // // // //   R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
// // // // // // //   return ans;
// // // // // // // }

// // // // // // // void Mission::step() //Mission::step()
// // // // // // // {
// // // // // // //   static int info_counter = 0;
// // // // // // //   geometry_msgs::Vector3     localOffset;

// // // // // // //   float speedFactor         = 2;
// // // // // // //   float yawThresholdInDeg   = 2;

// // // // // // //   float xCmd, yCmd, zCmd;

// // // // // // //   localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

// // // // // // //   double xOffsetRemaining = target_offset_x - localOffset.x;
// // // // // // //   double yOffsetRemaining = target_offset_y - localOffset.y;
// // // // // // //   double zOffsetRemaining = target_offset_z - localOffset.z;

// // // // // // //   double yawDesiredRad     = deg2rad * target_yaw;
// // // // // // //   double yawThresholdInRad = deg2rad * yawThresholdInDeg;
// // // // // // //   double yawInRad          = toEulerAngle(current_atti).z;

// // // // // // //   info_counter++;
// // // // // // //   if(info_counter > 25)
// // // // // // //   {
// // // // // // //     info_counter = 0;
// // // // // // //     ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
// // // // // // //     ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
// // // // // // //   }
// // // // // // //   if (abs(xOffsetRemaining) >= speedFactor)
// // // // // // //     xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
// // // // // // //   else
// // // // // // //     xCmd = xOffsetRemaining;

// // // // // // //   if (abs(yOffsetRemaining) >= speedFactor)
// // // // // // //     yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
// // // // // // //   else
// // // // // // //     yCmd = yOffsetRemaining;

// // // // // // //   zCmd = start_gps_location.altitude + target_offset_z;


// // // // // // //   /*!
// // // // // // //    * @brief: if we already started breaking, keep break for 50 sample (1sec)
// // // // // // //    *         and call it done, else we send normal command
// // // // // // //    */

// // // // // // //   if (break_counter > 50)
// // // // // // //   {
// // // // // // //     ROS_INFO("##### Route %d finished....", state);
// // // // // // //     finished = true;
// // // // // // //     return;
// // // // // // //   }
// // // // // // //   else if(break_counter > 0)
// // // // // // //   {
// // // // // // //     sensor_msgs::Joy controlVelYawRate;
// // // // // // //     uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
// // // // // // //                 DJISDK::HORIZONTAL_VELOCITY |
// // // // // // //                 DJISDK::YAW_RATE            |
// // // // // // //                 DJISDK::HORIZONTAL_GROUND   |
// // // // // // //                 DJISDK::STABLE_ENABLE);
// // // // // // //     controlVelYawRate.axes.push_back(0);
// // // // // // //     controlVelYawRate.axes.push_back(0);
// // // // // // //     controlVelYawRate.axes.push_back(0);
// // // // // // //     controlVelYawRate.axes.push_back(0);
// // // // // // //     controlVelYawRate.axes.push_back(flag);

// // // // // // //     ctrlBrakePub.publish(controlVelYawRate);
// // // // // // //     break_counter++;
// // // // // // //     return;
// // // // // // //   }
// // // // // // //   else //break_counter = 0, not in break stage
// // // // // // //   {
// // // // // // //     sensor_msgs::Joy controlPosYaw;


// // // // // // //     controlPosYaw.axes.push_back(xCmd);
// // // // // // //     controlPosYaw.axes.push_back(yCmd);
// // // // // // //     controlPosYaw.axes.push_back(zCmd);
// // // // // // //     controlPosYaw.axes.push_back(yawDesiredRad);
// // // // // // //     ctrlPosYawPub.publish(controlPosYaw);
// // // // // // //   }

// // // // // // //   if (std::abs(xOffsetRemaining) < 0.5 &&
// // // // // // //       std::abs(yOffsetRemaining) < 0.5 &&
// // // // // // //       std::abs(zOffsetRemaining) < 0.5 &&
// // // // // // //       std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
// // // // // // //   {
// // // // // // //     //! 1. We are within bounds; start incrementing our in-bound counter
// // // // // // //     inbound_counter ++;
// // // // // // //   }
// // // // // // //   else
// // // // // // //   {
// // // // // // //     if (inbound_counter != 0)
// // // // // // //     {
// // // // // // //       //! 2. Start incrementing an out-of-bounds counter
// // // // // // //       outbound_counter ++;
// // // // // // //     }
// // // // // // //   }

// // // // // // //   //! 3. Reset withinBoundsCounter if necessary
// // // // // // //   if (outbound_counter > 10)
// // // // // // //   {
// // // // // // //     ROS_INFO("##### Route %d: out of bounds, reset....", state);
// // // // // // //     inbound_counter  = 0;
// // // // // // //     outbound_counter = 0;
// // // // // // //   }

// // // // // // //   if (inbound_counter > 50)
// // // // // // //   {
// // // // // // //     ROS_INFO("##### Route %d start break....", state);
// // // // // // //     break_counter = 1;
// // // // // // //   }

// // // // // // // }

// // // // // // // bool takeoff_land(int task)
// // // // // // // {
// // // // // // //   dji_sdk::DroneTaskControl droneTaskControl;

// // // // // // //   droneTaskControl.request.task = task;

// // // // // // //   drone_task_service.call(droneTaskControl);

// // // // // // //   if(!droneTaskControl.response.result)
// // // // // // //   {
// // // // // // //     ROS_ERROR("takeoff_land fail");
// // // // // // //     return false;
// // // // // // //   }

// // // // // // //   return true;
// // // // // // // }

// // // // // // // bool obtain_control()
// // // // // // // {
// // // // // // //   dji_sdk::SDKControlAuthority authority;
// // // // // // //   authority.request.control_enable=1;
// // // // // // //   sdk_ctrl_authority_service.call(authority);

// // // // // // //   if(!authority.response.result)
// // // // // // //   {
// // // // // // //     ROS_ERROR("obtain control failed!");
// // // // // // //     return false;
// // // // // // //   }

// // // // // // //   return true;
// // // // // // // }

// // // // // // // bool is_M100()
// // // // // // // {
// // // // // // //   dji_sdk::QueryDroneVersion query;
// // // // // // //   query_version_service.call(query);

// // // // // // //   if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
// // // // // // //   {
// // // // // // //     return true;
// // // // // // //   }

// // // // // // //   return false;
// // // // // // // }

// // // // // // // void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
// // // // // // // {
// // // // // // //   current_atti = msg->quaternion;
// // // // // // // }

// // // // // // // void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
// // // // // // // {
// // // // // // //   static ros::Time start_time = ros::Time::now();
// // // // // // //   ros::Duration elapsed_time = ros::Time::now() - start_time;
// // // // // // //   current_gps = *msg;

// // // // // // //   // Down sampled to 50Hz loop
// // // // // // //   if(elapsed_time > ros::Duration(0.02))
// // // // // // //   {
// // // // // // //     start_time = ros::Time::now();
// // // // // // //     switch(square_mission.state)
// // // // // // //     {
// // // // // // //       case 0:
// // // // // // //         ROS_ERROR("Square mission end(maybe...); BREAK");
// // // // // // //         break;

// // // // // // //       case 1:
// // // // // // //         if(!square_mission.finished)
// // // // // // //         {
// // // // // // //           square_mission.step();
// // // // // // //         }
// // // // // // //         else
// // // // // // //         {
// // // // // // //           square_mission.reset();
// // // // // // //           square_mission.start_gps_location = current_gps;
// // // // // // //           square_mission.setTarget(20, 0, 0, 0);
// // // // // // //           square_mission.state = 2;
// // // // // // //           ROS_INFO("##### Start route %d ....", square_mission.state);
// // // // // // //           ROS_INFO("##### Start route %d ....", square_mission.state);
// // // // // // //         }
// // // // // // //         break;

// // // // // // //       case 2:
// // // // // // //         if(!square_mission.finished)
// // // // // // //         {
// // // // // // //           square_mission.step();
// // // // // // //         }
// // // // // // //         else
// // // // // // //         {
// // // // // // //           square_mission.reset();
// // // // // // //           square_mission.start_gps_location = current_gps;
// // // // // // //           square_mission.setTarget(0, -20, 0, 0);
// // // // // // //           square_mission.state = 3;
// // // // // // //           ROS_ERROR("##### Start route %d ....", square_mission.state);
// // // // // // //         }
// // // // // // //         break;
// // // // // // //       case 3:
// // // // // // //         if(!square_mission.finished)
// // // // // // //         {
// // // // // // //           square_mission.step();
// // // // // // //         }
// // // // // // //         else
// // // // // // //         {
// // // // // // //           square_mission.reset();
// // // // // // //           square_mission.start_gps_location = current_gps;
// // // // // // //           square_mission.setTarget(-20, 0, 0, 0);
// // // // // // //           square_mission.state = 4;
// // // // // // //           ROS_INFO("##### Start route %d ....", square_mission.state);
// // // // // // //         }
// // // // // // //         break;
// // // // // // //       case 4:
// // // // // // //         if(!square_mission.finished)
// // // // // // //         {
// // // // // // //           square_mission.step();
// // // // // // //         }
// // // // // // //         else
// // // // // // //         {
// // // // // // //           ROS_INFO("##### Mission %d Finished ....", square_mission.state);
// // // // // // //           square_mission.state = 0;
// // // // // // //         }
// // // // // // //         break;
// // // // // // //     }
// // // // // // //   }
// // // // // // // }

// // // // // // // void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
// // // // // // // {
// // // // // // //   flight_status = msg->data;
// // // // // // // }

// // // // // // // void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
// // // // // // // {
// // // // // // //   display_mode = msg->data;
// // // // // // // }


// // // // // // // /*!
// // // // // // //  * This function demos how to use the flight_status
// // // // // // //  * and the more detailed display_mode (only for A3/N3)
// // // // // // //  * to monitor the take off process with some error
// // // // // // //  * handling. Note M100 flight status is different
// // // // // // //  * from A3/N3 flight status.
// // // // // // //  */
// // // // // // // bool
// // // // // // // monitoredTakeoff()
// // // // // // // {
// // // // // // //   ros::Time start_time = ros::Time::now();

// // // // // // //   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
// // // // // // //     return false;
// // // // // // //   }

// // // // // // //   ros::Duration(0.01).sleep();
// // // // // // //   ros::spinOnce();

// // // // // // //   // Step 1.1: Spin the motor
// // // // // // //   while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
// // // // // // //          display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
// // // // // // //          ros::Time::now() - start_time < ros::Duration(5)) {
// // // // // // //     ros::Duration(0.01).sleep();
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   if(ros::Time::now() - start_time > ros::Duration(10)) {
// // // // // // //     ROS_ERROR("Takeoff failed. Motors are not spinnning.");
// // // // // // //     return false;
// // // // // // //   }
// // // // // // //   else {
// // // // // // //     start_time = ros::Time::now();
// // // // // // //     ROS_INFO("Motor Spinning ...");
// // // // // // //     ros::spinOnce();
// // // // // // //   }


// // // // // // //   // Step 1.2: Get in to the air
// // // // // // //   while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
// // // // // // //           (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
// // // // // // //           ros::Time::now() - start_time < ros::Duration(10)) {
// // // // // // //     ros::Duration(0.01).sleep();
// // // // // // //     //ROS_INFO("Waiting...");
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   if(ros::Time::now() - start_time > ros::Duration(10)) {
// // // // // // //     ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
// // // // // // //     ROS_ERROR("But we will ignore this problem.");
// // // // // // //     //return false;
// // // // // // //   }
// // // // // // //   else {
// // // // // // //     start_time = ros::Time::now();
// // // // // // //     ROS_INFO("Ascending...");
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   // Final check: Finished takeoff
// // // // // // //   while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
// // // // // // //           ros::Time::now() - start_time < ros::Duration(10)) {
// // // // // // //     ros::Duration(0.01).sleep();
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
// // // // // // //   {
// // // // // // //     ROS_INFO("Successful takeoff!");
// // // // // // //     start_time = ros::Time::now();
// // // // // // //   }
// // // // // // //   else
// // // // // // //   {
// // // // // // //     ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
// // // // // // //     return false;
// // // // // // //   }

// // // // // // //   return true;
// // // // // // // }


// // // // // // // /*!
// // // // // // //  * This function demos how to use M100 flight_status
// // // // // // //  * to monitor the take off process with some error
// // // // // // //  * handling. Note M100 flight status is different
// // // // // // //  * from A3/N3 flight status.
// // // // // // //  */
// // // // // // // bool
// // // // // // // M100monitoredTakeoff()
// // // // // // // {
// // // // // // //   ros::Time start_time = ros::Time::now();

// // // // // // //   float home_altitude = current_gps.altitude;
// // // // // // //   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
// // // // // // //   {
// // // // // // //     return false;
// // // // // // //   }

// // // // // // //   ros::Duration(0.01).sleep();
// // // // // // //   ros::spinOnce();

// // // // // // //   // Step 1: If M100 is not in the air after 10 seconds, fail.
// // // // // // //   while (ros::Time::now() - start_time < ros::Duration(10))
// // // // // // //   {
// // // // // // //     ros::Duration(0.01).sleep();
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
// // // // // // //       current_gps.altitude - home_altitude < 1.0)
// // // // // // //   {
// // // // // // //     ROS_ERROR("Takeoff failed.");
// // // // // // //     return false;
// // // // // // //   }
// // // // // // //   else
// // // // // // //   {
// // // // // // //     start_time = ros::Time::now();
// // // // // // //     ROS_INFO("Successful takeoff!");
// // // // // // //     ros::spinOnce();
// // // // // // //   }

// // // // // // //   return true;
// // // // // // // }


// // // // // // // csv test
// // // // // // #include <iostream>
// // // // // // #include <fstream>

// // // // // // using namespace std;

// // // // // // int main(){
// // // // // //   ifstream myFile;
// // // // // //   myFile.open("/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/src/test.csv");
// // // // // //   cout << 1 << endl;
// // // // // //   cout << myFile.good() << "\n" << endl;

// // // // // //   while(myFile.good()){
// // // // // //     string line;
// // // // // //     getline(myFile, line, ',');
// // // // // //     cout << line << endl;
// // // // // //   }
// // // // // // }


// // // // // /// gimbal test code ///
// // // // // #include "ros/ros.h"
// // // // // #include "std_msgs/Float64MultiArray.h"

// // // // // using namespace std;

// // // // // int main(int argc, char **argv){

// // // // //   ros::init(argc, argv, "test");
// // // // //   ros::NodeHandle n;
  
// // // // //   // ROS_INFO("Wait 2s");
// // // // //   // ros::Duration(2).sleep();
  
// // // // //   ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
// // // // //   cout << "check point 1" << endl;
// // // // //   std_msgs::Float64MultiArray gim;
// // // // //   cout << "check point 2" << endl;

// // // // //   /*
// // // // //   상하로 제어할 때는 56~77까지이다.
// // // // //   56일때 아래, 77일때 정면

// // // // //   짐벌 좌우로 제어 가능한 범위는 56~96이다.
// // // // //   56일때 드론 기준 왼쪽, 96일때 드론 기준 오른쪽
// // // // //   */
// // // // //   gim.data.push_back(50); // 상하
// // // // //   gim.data.push_back(0); // 좌우
// // // // //   gimbal_control.publish(gim); // 제어 명령 보내기
// // // // //   cout << "check point 3" << endl;
// // // // //   // ros::Duration(3).sleep();

// // // // //   gim.data[0] = 50;
// // // // //   cout << "check point 4" << endl;
// // // // //   gim.data[1] = 100;
// // // // //   gimbal_control.publish(gim);
// // // // //   cout << "50,100" << endl;
// // // // //   ros::Duration(5).sleep();

// // // // //   gim.data[0] = 0;
// // // // //   gim.data[1] = 50;
// // // // //   gimbal_control.publish(gim);
// // // // //   cout << "0,50" << endl;
// // // // //   ros::Duration(5).sleep();
// // // // //   cout << "check point 5" << endl;


// // // // //   for(int i = 0;i < 100; i++){
// // // // //     gim.data[0] = i;
// // // // //     gim.data[1] = i;
// // // // //     gimbal_control.publish(gim);
// // // // //     cout << i << ",0" << endl;
// // // // //     ros::Duration(0.5).sleep();
// // // // //   }
  
// // // // //   return 0;
// // // // // }


// // // // /// control gimbal with apriltag ///
// // // #include "ros/ros.h"
// // // #include "std_msgs/Float64MultiArray.h"
// // // #include "tf2_msgs/TFMessage.h"
// // // // multi thread
// // // #include <mutex>
// // // #include <thread>
// // // #include <vector>
// // // #include <chrono>
// // // #include <string>

// // // #include<unistd.h>
// // // #include<termios.h>

// // // #include<math.h>

// // // using namespace std;

// // // int angle_gimbal; // 각도 전역변수
// // // float x_tf;
// // // float y_tf;
// // // float z_tf;
// // // float gimbal_x;
// // // float gimbal_y;
// // // float chk_x;
// // // float chk_y;
// // // double ang_x;
// // // double ang_y;
// // // bool is_run_thread;

// // // int getch();

// // // void callback_tf(const tf2_msgs::TFMessage msg);
// // // std_msgs::Float64MultiArray gim;


// // // int main(int argc, char **argv){

// // //   cout << "Start test_code" << endl;
// // //   ROS_INFO("Start test_code");
// // //   is_run_thread = true;

// // //   // // 역삼각함수 확인
// // //   // double x,ret;

// // //   // x = 1;
// // //   // ret = atan(x); 
// // //   // printf("atan( %lf ) = %lf\n", x, ret);
// // //   // cout << M_PI << endl;
// // //   // ret = ret * 180/M_PI;
// // //   // cout << ret << endl;

// // //   // return 0;
// // //   // // 삼각함수 확인 여기까지

// // //   ros::init(argc, argv, "test");
// // //   ros::NodeHandle n;
    
// // //   ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
// // //   std_msgs::Float64MultiArray gim;

// // //   // 여기서부터 정보 받고, 제어하는 코드
// // //   ros::Subscriber sub_tf = n.subscribe("tf", 1000, callback_tf); // tf 값 수신

// // //   auto ros_spin = []()
// // // 	{
// // //     cout << "Start ros spin" << endl;
// // // 		ros::spin();
// // //     is_run_thread = false;
// // // 	};

// // //   // 쓰레드 2 - Control gimbal
// // //   auto control_drone = []()
// // // 	{
// // //     /*
// // //     상하로 제어할 때는 56~77까지이다.
// // //     56일때 아래, 77일때 정면

// // //     짐벌 좌우로 제어 가능한 범위는 56~96이다.
// // //     56일때 드론 기준 왼쪽, 96일때 드론 기준 오른쪽
// // //     */

// // //     cout << "start gimbal control" << endl;
		
// // //     ros::NodeHandle n;
// // //     ros::Publisher gimbal_control = n.advertise<std_msgs::Float64MultiArray>("gimbal_control", 10);
// // //     std_msgs::Float64MultiArray gim; // 위에 있어서 일단 주석처리 했음. 안되면 주석 풀기
// // //     gim.data.push_back(77); // 상하 // 위에서 이미 해서 지웠음.
// // //     gim.data.push_back(65); // 좌우
// // //     gimbal_control.publish(gim);
// // //     ros::Duration(1).sleep();

// // //     gimbal_x = 76;
// // //     gimbal_y = 77;
// // //     x_tf = 0;
// // //     y_tf = 0;
// // //     chk_x = 0;
// // //     chk_y = 0;
// // //     gim.data[0] = gimbal_y;
// // //     gim.data[1] = gimbal_x;
// // //     gimbal_control.publish(gim);
// // //     ros::Duration(1).sleep();

// // //     while(is_run_thread){
// // //       // gimbal x 방향 회전 (pan // 65~87) 각도 단위는 8도
// // //       if(chk_x != x_tf){

// // //         // if(x_tf > 0.05){// 이거는 확인해보기
// // //         //   gimbal_x = gimbal_x + 1;
// // //         // }
// // //         // else if(x_tf < -0.05){
// // //         //   gimbal_x = gimbal_x - 1;
// // //         // }
// // //         int x_threshold = 16; // 8
// // //         ang_x = atan(x_tf/z_tf)*180/M_PI;
// // //         if (abs(ang_x) > x_threshold - 1){
// // //           gimbal_x = gimbal_x + ang_x/x_threshold;
// // //         }
        
// // //         if(gimbal_x > 87){
// // //           gimbal_x = 87;
// // //         }
// // //         else if(gimbal_x < 65){
// // //           gimbal_x = 65;
// // //         }      

// // //         chk_x = x_tf;
// // //       }

// // //       // gimbal y 방향 회전 (tilt // 56 ~ 77(77이 정면이고, 더 안올라감)) 각도 단위는 4.286도
// // //       if(chk_y != y_tf){
// // //         // if(y_tf > 0.05){// 이거는 확인해보기
// // //         //   gimbal_y = gimbal_y - 1;
// // //         // }
// // //         // else if(y_tf < -0.05){
// // //         //   gimbal_y = gimbal_y + 1;
// // //         // }
// // //         int y_threshold = 8; // 4
// // //         ang_y = atan(y_tf/z_tf)*180/M_PI;
// // //         if(abs(ang_y) > y_threshold - 1){
// // //           gimbal_y = gimbal_y - ang_y / y_threshold;
// // //         }

// // //         if(gimbal_y > 79){
// // //             gimbal_y = 79;
// // //         }
// // //         else if(gimbal_y < 56){
// // //           gimbal_y = 56;
// // //         }
// // //         chk_y = y_tf;
// // //       }

// // //       gim.data[1] = gimbal_x;
// // //       gim.data[0] = gimbal_y;

// // //       // cout << gim << endl;
// // //       gimbal_control.publish(gim); // 값이 바뀌는것과 상관 없이 같은 값 publish
      
// // //       ros::Duration(0.3).sleep(); // sleep for hundredth of a second
// // //     }
// // // 	};

// // //   // 쓰레드 3 - Get button input
// // //   auto get_button = [](){
// // //     cout << "start get button" << endl;
// // //     while(is_run_thread){
// // //       int c1 = getch();    // 입력 버퍼를 사용하지 않음, 화면에 키의 입력을 보여주지 않음
// // //       cout << "Key input : " << c1 << endl;
// // //       if(c1 == 99){ // "C 입력시 종료"
// // //         is_run_thread = false;
// // //       }
// // //     }
// // //   };

// // //   std::thread t1 = std::thread(ros_spin);
// // // 	std::thread t2 = std::thread(control_drone);
// // //   std::thread t3 = std::thread(get_button);
// // //   t1.join(); // 쓰레드 끝날때 까지 main 함수 종료 안함.
// // // 	t2.join();
// // //   // t3.join();
  
// // //   return 0;
// // // }

// // // void callback_tf(const tf2_msgs::TFMessage msg) // need to be edited
// // // {
// // //   x_tf = msg.transforms[0].transform.translation.x;
// // //   y_tf = msg.transforms[0].transform.translation.y;
// // //   z_tf = msg.transforms[0].transform.translation.z;
// // // }

// // // int getch()
// // // {
// // //   int c;
// // //   struct termios oldattr, newattr;

// // //   tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
// // //   newattr = oldattr;
// // //   newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
// // //   newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
// // //   newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
// // //   tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
// // //   c = getchar();                               // 키보드 입력 읽음
// // //   tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
// // //   return c;
// // // }


// // // // 현재 시간 기록하는 방법?
// // #include <time.h>
// // #include <iostream>

// // #include "ros/ros.h"
// // #include "std_msgs/Float64MultiArray.h"
// // #include "tf2_msgs/TFMessage.h"

// // using namespace std;

// // void get_time();
// // int year, month, day, hour, minute, sec;

// // int main(){

// //   get_time();
// //   cout << minute << endl;
// //   cout << sec << endl;
  
// //   return 0;
// // }

// // void get_time(){
// //   time_t rawTime;
// //   struct tm* pTimeInfo;

// //   rawTime = time(NULL);
// //   pTimeInfo = localtime(&rawTime);

// //   // float time_raw = pTimeInfo;
// //   for(int i=0; i < 10; i++){
// //     cout << rawTime << endl;

// //   }
  
// //   year = pTimeInfo->tm_year + 1900;    //연도에는 1900 더해줌
// //   month = pTimeInfo->tm_mon + 1;    // 월에는 1 더해줌
// //   day = pTimeInfo->tm_mday;
// //   hour = pTimeInfo->tm_hour;
// //   minute = pTimeInfo->tm_min;
// //   sec = pTimeInfo->tm_sec;
// // }


// #include <chrono>
// #include <iostream>
// #include <sys/time.h>
// #include <ctime>

// using std::cout; using std::endl;

// int main() {
//   for(int i = 0; i < 10; i++){
//     time_t now = time(nullptr);
//     time_t mnow = now * 1000;

//     cout << "seconds since epoch: " << now << endl;
//     cout << "milliseconds since epoch: " << mnow << endl << endl;
//   }
//     return EXIT_SUCCESS;
// }

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

// using std::cout; using std::endl;
using namespace std;

int main() {
  
  struct timeval time_now{};
  for(int i =0; i<100; i++){
    gettimeofday(&time_now, nullptr);
    time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

    cout << "seconds since epoch: " << time_now.tv_sec << endl;
    cout << "milliseconds since epoch: "  << msecs_time << endl << endl;
  }
  return EXIT_SUCCESS;
}

// int main() {
//   for(int i =0; i<100; i++){
//     struct timeval time_now{};
//     gettimeofday(&time_now, nullptr);
//     time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

//     cout << "seconds since epoch: " << time_now.tv_sec << endl;
//     cout << "milliseconds since epoch: "  << msecs_time << endl << endl;
//   }
//   return EXIT_SUCCESS;
// }