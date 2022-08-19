#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/**
#include <dji_sdk/dji_sdk_node.h>
**/

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
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
/*
void get_vel(const geometry_msgs::Vector3Stamped vel)
{
  ROS_INFO("Velocity: [%f]", vel[1]);
  //ROS_INFO("Velocity: [%f]", vel.vector[1]);
  //ROS_INFO("Velocity: [%f]", vel.vector[2]);
}
*/

/**
void chatterCallback_practice(const )
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
**/

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
  ros::init(argc, argv, "listener_ros");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n; //노드 핸들은 한개만 있어도 여러개 subscribe가 가능한 것 같다.
  //ros::NodeHandle n2;
  //ros::NodeHandle n3;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber sub = n.subscribe("chatter_ros", 1000, chatterCallback);
  ros::Subscriber sub_imu = n.subscribe("dji_sdk/imu", 1000, chatterCallback2);
//  ros::Subscriber sub3 = n3.subscribe("dji_sdk/angular_velocity_fused", 1000, get_vel); //n3 대신에 n으로 해도 되는거 같다. - 확인 필요
/**  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback_practice);
**/

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
