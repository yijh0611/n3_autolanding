#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dji_sdk/DroneArmControl.h"

#include <sstream>
// global variables
//ros::ServiceClient arm_service_tst;

ros::NodeHandle nh;

ros::ServiceClient arm_service_tst;

/*
#include <djiosdk/dji_status.hpp>/*
#include <djiosdk/dji_control.hpp>

#include <djiosdk/dji_version.hpp>
*/


bool arm_tst(int arm)
{
  dji_sdk::DroneArmControl droneArmControl;

  droneArmControl.request.arm = arm;

  drone_arm_service.call(droneArmControl);

  if(!droneArmControl.response.result)
  {
    ROS_ERROR("Arm fail");
    return false;
  }

  return true;
}

/*
void
ServiceAck
//arm(dji_sdk::DroneArmControl& droneArm)
arm()
{
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 1;
  drone_arm_service_tst.call(droneArmControl);
  if (!droneArmControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
  }
  return ServiceAck(
    droneArmControl.response.result, droneArmControl.response.cmd_set,
    droneArmControl.response.cmd_id, droneArmControl.response.ack_data);
}
*/


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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_ros", 1000);
  arm_service_tst = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  //arm_service_tst = n.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/DroneArmControl");

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    if(0){
      ss << "hello world ros " << count ;/* << DJI::OSDK::Version::M100_31; /**<< DJI::OSDK::Status;*/
    }
    else{
      ss << "hello world ros " << count;
    }
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    arm_tst(1);
  }


  return 0;
}
