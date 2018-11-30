/*
*   Rosnode to publish uuv-Positions
*   Nils Timmermann
*   Nils.Timmermann@tuhh.de
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped pose3;
geometry_msgs::PoseStamped pose4;
geometry_msgs::PoseStamped pose5;
nav_msgs::Path path1;
nav_msgs::Path path1b;
nav_msgs::Path path2;
nav_msgs::Path path3;
nav_msgs::Path path4;
nav_msgs::Path path5;
int steps1 = 1;
int steps2 = 1;
int steps3 = 1;
int steps4 = 1;
int steps5 = 1;

/*void append(const geometry_msgs::PoseStamped::ConstPtr& position,const nav_msgs::Path::ConstPtr& path, int path_length){
    geometry_msgs::PoseStamped poses[path_length];
    for(int i=0;i<path_length-1;i++){
        poses[i]=path[i].pose;
    }
    poses[path_length-1]=position;
    path.poses = poses;

}*/

void Pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose1.header = msg->header;
    path1.header = msg->header;
    pose1.pose.position.x = msg->pose.position.x;
    pose1.pose.position.y = msg->pose.position.y;
    pose1.pose.position.z = msg->pose.position.z;
    pose1.pose.orientation.x = msg->pose.orientation.x;
    pose1.pose.orientation.y = msg->pose.orientation.y;
    pose1.pose.orientation.z = msg->pose.orientation.z;
    pose1.pose.orientation.w = msg->pose.orientation.w;

    std::vector<geometry_msgs::PoseStamped> poses(steps1);
    for(int i=0;i<(steps1-1);i++){
        poses.at(i)=path1.poses[i];
    }
    poses.at(steps1-1) = pose1;
    path1.poses = poses;
    steps1++;
}

void Pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose2.header = msg->header;
    path2.header = msg->header;
    pose2.pose.position.x = msg->pose.position.x+1;
    pose2.pose.position.y = msg->pose.position.y;
    pose2.pose.position.z = msg->pose.position.z;
    pose2.pose.orientation.x = msg->pose.orientation.x;
    pose2.pose.orientation.y = msg->pose.orientation.y;
    pose2.pose.orientation.z = msg->pose.orientation.z;
    pose2.pose.orientation.w = msg->pose.orientation.w;

    std::vector<geometry_msgs::PoseStamped> poses(steps2);
    for(int i=0;i<(steps2-1);i++){
        poses.at(i)=path2.poses[i];
    }
    poses.at(steps2-1) = pose2;
    path2.poses = poses;
    steps2++;
}

void Pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose3.header = msg->header;
    path3.header = msg->header;
    pose3.pose.position.x = msg->pose.position.x+2;
    pose3.pose.position.y = msg->pose.position.y;
    pose3.pose.position.z = msg->pose.position.z;
    pose3.pose.orientation.x = msg->pose.orientation.x;
    pose3.pose.orientation.y = msg->pose.orientation.y;
    pose3.pose.orientation.z = msg->pose.orientation.z;
    pose3.pose.orientation.w = msg->pose.orientation.w;

    std::vector<geometry_msgs::PoseStamped> poses(steps3);
    for(int i=0;i<(steps3-1);i++){
        poses.at(i)=path3.poses[i];
    }
    poses.at(steps3-1) = pose3;
    path3.poses = poses;
    steps3++;
}

void Pose4Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose4.header = msg->header;
    path4.header = msg->header;
    pose4.pose.position.x = msg->pose.position.x+3;
    pose4.pose.position.y = msg->pose.position.y;
    pose4.pose.position.z = msg->pose.position.z;
    pose4.pose.orientation.x = msg->pose.orientation.x;
    pose4.pose.orientation.y = msg->pose.orientation.y;
    pose4.pose.orientation.z = msg->pose.orientation.z;
    pose4.pose.orientation.w = msg->pose.orientation.w;

    std::vector<geometry_msgs::PoseStamped> poses(steps4);
    for(int i=0;i<(steps4-1);i++){
        poses.at(i)=path4.poses[i];
    }
    poses.at(steps4-1) = pose4;
    path4.poses = poses;
    steps4++;
}

void Pose5Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose5.header = msg->header;
    path5.header = msg->header;
    pose5.pose.position.x = msg->pose.position.x+4;
    pose5.pose.position.y = msg->pose.position.y;
    pose5.pose.position.z = msg->pose.position.z;
    pose5.pose.orientation.x = msg->pose.orientation.x;
    pose5.pose.orientation.y = msg->pose.orientation.y;
    pose5.pose.orientation.z = msg->pose.orientation.z;
    pose5.pose.orientation.w = msg->pose.orientation.w;

    std::vector<geometry_msgs::PoseStamped> poses(steps5);
    for(int i=0;i<(steps5-1);i++){
        poses.at(i)=path5.poses[i];
    }
    poses.at(steps5-1) = pose5;
    path5.poses = poses;
    steps5++;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "uuv_com");
    ros::NodeHandle n;

    ros::Subscriber sub1_pose = n.subscribe("uuv1/mavros/local_position/pose", 1, Pose1Callback);
    ros::Subscriber sub2_pose = n.subscribe("uuv2/mavros/local_position/pose", 1, Pose2Callback);
    ros::Subscriber sub3_pose = n.subscribe("uuv3/mavros/local_position/pose", 1, Pose3Callback);
    ros::Subscriber sub4_pose = n.subscribe("uuv4/mavros/local_position/pose", 1, Pose4Callback);
    ros::Subscriber sub5_pose = n.subscribe("uuv5/mavros/local_position/pose", 1, Pose5Callback);
    ros::Publisher pub11_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/UUV1Pose/pose", 1);
    ros::Publisher pub12_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/UUV1Pose/pose", 1);
    ros::Publisher pub13_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/UUV1Pose/pose", 1);
    ros::Publisher pub14_pose = n.advertise<geometry_msgs::PoseStamped>("uuv4/mavros/UUV1Pose/pose", 1);
    ros::Publisher pub15_pose = n.advertise<geometry_msgs::PoseStamped>("uuv5/mavros/UUV1Pose/pose", 1);

    ros::Publisher pub21_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/UUV2Pose/pose", 1);
    ros::Publisher pub22_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/UUV2Pose/pose", 1);
    ros::Publisher pub23_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/UUV2Pose/pose", 1);
    ros::Publisher pub24_pose = n.advertise<geometry_msgs::PoseStamped>("uuv4/mavros/UUV2Pose/pose", 1);
    ros::Publisher pub25_pose = n.advertise<geometry_msgs::PoseStamped>("uuv5/mavros/UUV2Pose/pose", 1);

    ros::Publisher pub31_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/UUV3Pose/pose", 1);
    ros::Publisher pub32_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/UUV3Pose/pose", 1);
    ros::Publisher pub33_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/UUV3Pose/pose", 1);
    ros::Publisher pub34_pose = n.advertise<geometry_msgs::PoseStamped>("uuv4/mavros/UUV3Pose/pose", 1);
    ros::Publisher pub35_pose = n.advertise<geometry_msgs::PoseStamped>("uuv5/mavros/UUV3Pose/pose", 1);

    ros::Publisher pub41_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/UUV4Pose/pose", 1);
    ros::Publisher pub42_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/UUV4Pose/pose", 1);
    ros::Publisher pub43_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/UUV4Pose/pose", 1);
    ros::Publisher pub44_pose = n.advertise<geometry_msgs::PoseStamped>("uuv4/mavros/UUV4Pose/pose", 1);
    ros::Publisher pub45_pose = n.advertise<geometry_msgs::PoseStamped>("uuv5/mavros/UUV4Pose/pose", 1);

    ros::Publisher pub51_pose = n.advertise<geometry_msgs::PoseStamped>("uuv1/mavros/UUV5Pose/pose", 1);
    ros::Publisher pub52_pose = n.advertise<geometry_msgs::PoseStamped>("uuv2/mavros/UUV5Pose/pose", 1);
    ros::Publisher pub53_pose = n.advertise<geometry_msgs::PoseStamped>("uuv3/mavros/UUV5Pose/pose", 1);
    ros::Publisher pub54_pose = n.advertise<geometry_msgs::PoseStamped>("uuv4/mavros/UUV5Pose/pose", 1);
    ros::Publisher pub55_pose = n.advertise<geometry_msgs::PoseStamped>("uuv5/mavros/UUV5Pose/pose", 1);

    ros::Publisher pub11_path = n.advertise<nav_msgs::Path>("uuv1/mavros/UUV1Path/path", 1);
    ros::Publisher pub12_path = n.advertise<nav_msgs::Path>("uuv2/mavros/UUV1Path/path", 1);
    ros::Publisher pub13_path = n.advertise<nav_msgs::Path>("uuv3/mavros/UUV1Path/path", 1);
    ros::Publisher pub14_path = n.advertise<nav_msgs::Path>("uuv4/mavros/UUV1Path/path", 1);
    ros::Publisher pub15_path = n.advertise<nav_msgs::Path>("uuv5/mavros/UUV1Path/path", 1);

    ros::Publisher pub21_path = n.advertise<nav_msgs::Path>("uuv1/mavros/UUV2Path/path", 1);
    ros::Publisher pub22_path = n.advertise<nav_msgs::Path>("uuv2/mavros/UUV2Path/path", 1);
    ros::Publisher pub23_path = n.advertise<nav_msgs::Path>("uuv3/mavros/UUV2Path/path", 1);
    ros::Publisher pub24_path = n.advertise<nav_msgs::Path>("uuv4/mavros/UUV2Path/path", 1);
    ros::Publisher pub25_path = n.advertise<nav_msgs::Path>("uuv5/mavros/UUV2Path/path", 1);

    ros::Publisher pub31_path = n.advertise<nav_msgs::Path>("uuv1/mavros/UUV3Path/path", 1);
    ros::Publisher pub32_path = n.advertise<nav_msgs::Path>("uuv2/mavros/UUV3Path/path", 1);
    ros::Publisher pub33_path = n.advertise<nav_msgs::Path>("uuv3/mavros/UUV3Path/path", 1);
    ros::Publisher pub34_path = n.advertise<nav_msgs::Path>("uuv4/mavros/UUV3Path/path", 1);
    ros::Publisher pub35_path = n.advertise<nav_msgs::Path>("uuv5/mavros/UUV3Path/path", 1);

    ros::Publisher pub41_path = n.advertise<nav_msgs::Path>("uuv1/mavros/UUV4Path/path", 1);
    ros::Publisher pub42_path = n.advertise<nav_msgs::Path>("uuv2/mavros/UUV4Path/path", 1);
    ros::Publisher pub43_path = n.advertise<nav_msgs::Path>("uuv3/mavros/UUV4Path/path", 1);
    ros::Publisher pub44_path = n.advertise<nav_msgs::Path>("uuv4/mavros/UUV4Path/path", 1);
    ros::Publisher pub45_path = n.advertise<nav_msgs::Path>("uuv5/mavros/UUV4Path/path", 1);

    ros::Publisher pub51_path = n.advertise<nav_msgs::Path>("uuv1/mavros/UUV5Path/path", 1);
    ros::Publisher pub52_path = n.advertise<nav_msgs::Path>("uuv2/mavros/UUV5Path/path", 1);
    ros::Publisher pub53_path = n.advertise<nav_msgs::Path>("uuv3/mavros/UUV5Path/path", 1);
    ros::Publisher pub54_path = n.advertise<nav_msgs::Path>("uuv4/mavros/UUV5Path/path", 1);
    ros::Publisher pub55_path = n.advertise<nav_msgs::Path>("uuv5/mavros/UUV5Path/path", 1);





    ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    pub11_pose.publish(pose1);
    pub12_pose.publish(pose1);
    pub13_pose.publish(pose1);
    pub14_pose.publish(pose1);
    pub15_pose.publish(pose1);

    pub21_pose.publish(pose2);
    pub22_pose.publish(pose2);
    pub23_pose.publish(pose2);
    pub24_pose.publish(pose2);
    pub25_pose.publish(pose2);

    pub31_pose.publish(pose3);
    pub32_pose.publish(pose3);
    pub33_pose.publish(pose3);
    pub34_pose.publish(pose3);
    pub35_pose.publish(pose3);

    pub41_pose.publish(pose4);
    pub42_pose.publish(pose4);
    pub43_pose.publish(pose4);
    pub44_pose.publish(pose4);
    pub45_pose.publish(pose4);

    pub51_pose.publish(pose5);
    pub52_pose.publish(pose5);
    pub53_pose.publish(pose5);
    pub54_pose.publish(pose5);
    pub55_pose.publish(pose5);

    pub11_path.publish(path1);
    pub12_path.publish(path1);
    pub13_path.publish(path1);
    pub14_path.publish(path1);
    pub15_path.publish(path1);

    pub21_path.publish(path2);
    pub22_path.publish(path2);
    pub23_path.publish(path2);
    pub24_path.publish(path2);
    pub25_path.publish(path2);

    pub31_path.publish(path3);
    pub32_path.publish(path3);
    pub33_path.publish(path3);
    pub34_path.publish(path3);
    pub35_path.publish(path3);

    pub41_path.publish(path4);
    pub42_path.publish(path4);
    pub43_path.publish(path4);
    pub44_path.publish(path4);
    pub45_path.publish(path4);

    pub51_path.publish(path5);
    pub52_path.publish(path5);
    pub53_path.publish(path5);
    pub54_path.publish(path5);
    pub55_path.publish(path5);



    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
