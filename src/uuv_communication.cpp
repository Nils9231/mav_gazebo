/*
*   Rosnode to publish uuv-Positions
*   Nils Timmermann
*   Nils.Timmermann@tuhh.de
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros/mavros_plugin.h>

geometry_msgs::PoseStamped msg_out;
mavros_msgs::PositionTarget pos_out;

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  msg_out.pose.position.x = msg->pose.position.x;
  msg_out.pose.position.y = msg->pose.position.y;
  msg_out.pose.position.z = msg->pose.position.z;

  /*pos_out.position.x = msg->pose.position.x;
  pos_out.position.y = msg->pose.position.y;
  pos_out.position.z = msg->pose.position.z;
*/
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "uuv_com");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("uav1/mavros/local_position/pose", 1000, PoseCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local/", 1000);
  //ros::Publisher pub_pos = n.advertise<mavros_msgs::PositionTarget>("uav2/mavros/setpoint_position/local/",1000);
  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    pub.publish(msg_out);
    //pub_pos.publish(pos_out);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
