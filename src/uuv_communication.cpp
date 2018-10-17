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
//#include <mavros/mavros_plugin.h>

geometry_msgs::PoseStamped msg_out1;
geometry_msgs::PoseStamped msg_out2;
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
void SpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  speed_out.linear.x = msg->twist.linear.x;
  speed_out.linear.y = msg->twist.linear.y;
  speed_out.linear.z = msg->twist.linear.z;


}
*/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "uuv_com");
  ros::NodeHandle n;
  ros::Subscriber sub1_pose = n.subscribe("uav1/mavros/local_position/pose", 1, Pose1Callback);
  //ros::Subscriber sub1_speed = n.subscribe("uav1/mavros/local_position/velocity", 1000, Pose1Callback);
  ros::Publisher pub1_pose = n.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local/", 1);
  //ros::Publisher pub1_speed = n.advertise<geometry_msgs::Twist>("/uav2/mavros/setpoint_velocity/cmd_vel_unstamped", 1000);
  ros::Subscriber sub2_pose = n.subscribe("uav2/mavros/local_position/pose", 1, Pose2Callback);
  //ros::Subscriber sub2_speed = n.subscribe("uav2/mavros/local_position/velocity", 1000, Pose1Callback);
  ros::Publisher pub2_pose = n.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local/", 1);
  //ros::Publisher pub2_speed = n.advertise<geometry_msgs::Twist>("/uav1/mavros/setpoint_velocity/cmd_vel_unstamped", 1000);
  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    pub1_pose.publish(msg_out1);
    pub2_pose.publish(msg_out2);
    //pub_speed.publish(speed_out);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
