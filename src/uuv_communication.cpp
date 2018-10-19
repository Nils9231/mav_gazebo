/*
*   Rosnode to publish uuv-Positions
*   Nils Timmermann
*   Nils.Timmermann@tuhh.de
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
//#include <mavros/mavros_plugin.h>

geometry_msgs::PoseStamped msg_out1;
geometry_msgs::PoseStamped msg_out2;
geometry_msgs::PoseStamped msg_out3;
mavros_msgs::GlobalPositionTarget glob1;
mavros_msgs::GlobalPositionTarget glob2;
mavros_msgs::GlobalPositionTarget glob3;
//geometry_msgs::Twist speed_out;

void Pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  msg_out1.pose.position.x = msg->pose.position.x;
  msg_out1.pose.position.y = msg->pose.position.y;
  msg_out1.pose.position.z = msg->pose.position.z;
  msg_out1.pose.orientation.x = msg->pose.orientation.x;
  msg_out1.pose.orientation.y = msg->pose.orientation.y;
  msg_out1.pose.orientation.z = msg->pose.orientation.z;
  msg_out1.pose.orientation.w = msg->pose.orientation.w;

}

void Pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  msg_out2.pose.position.x = msg->pose.position.x;
  msg_out2.pose.position.y = msg->pose.position.y;
  msg_out2.pose.position.z = msg->pose.position.z;
  msg_out2.pose.orientation.x = msg->pose.orientation.x;
  msg_out2.pose.orientation.y = msg->pose.orientation.y;
  msg_out2.pose.orientation.z = msg->pose.orientation.z;
  msg_out2.pose.orientation.w = msg->pose.orientation.w;

}
/*
void Pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  msg_out3.pose.position.x = msg->pose.position.x;
  msg_out3.pose.position.y = msg->pose.position.y;
  msg_out3.pose.position.z = msg->pose.position.z;
  msg_out3.pose.orientation.x = msg->pose.orientation.x;
  msg_out3.pose.orientation.y = msg->pose.orientation.y;
  msg_out3.pose.orientation.z = msg->pose.orientation.z;
  msg_out3.pose.orientation.w = msg->pose.orientation.w;

}
*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "uuv_com");
  ros::NodeHandle n;

  ros::Subscriber sub1_pose = n.subscribe("uuv1/mavros/local_position/pose", 1, Pose1Callback);
  ros::Subscriber sub2_pose = n.subscribe("uuv2/mavros/local_position/pose", 1, Pose2Callback);
  //ros::Subscriber sub3_pose = n.subscribe("uuv3/mavros/local_position/pose", 1, Pose3Callback);
  ros::Publisher pub12_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/setpoint_position/local/", 1);
  ros::Publisher pub13_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/setpoint_position/local/", 1);
  ros::Publisher pub21_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/setpoint_position/local/", 1);
  //ros::Publisher pub23_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/setpoint_position/local/", 1);
  //ros::Publisher pub31_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/setpoint_position/local/", 1);
  //ros::Publisher pub32_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/setpoint_position/local/", 1);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    pub12_pose.publish(msg_out1);
    //pub13_pose.publish(msg_out1);
    pub21_pose.publish(msg_out2);
    //pub23_pose.publish(msg_out2);
    //pub31_pose.publish(msg_out3);
    //pub32_pose.publish(msg_out3);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
