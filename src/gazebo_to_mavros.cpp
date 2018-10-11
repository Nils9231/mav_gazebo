/*
*   gazebo_to_mavros node to get information such as the position from gazebo transported to the PX4 Firmware
*   Nils Rottmann
*   Nils.Rottmann@tuhh.de
*/


#include <string>
#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>                // including the required messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandLong.h>



/*
This topic subscribes to the gazebo ros topic /gazebo/model_states to get the state of the different models in the gazebo simulation
and publishes it to the /mavros/mocap/pose topic of the mavros package.
*/

//using global coordinates to store the subscribed data
geometry_msgs::PoseStamped msg_out;
geometry_msgs::PoseWithCovarianceStamped msg_out_cov;
nav_msgs::Odometry odom;
mavros_msgs::HomePosition home;
ros::Time current_time;


// Global variables to find the model number of the array
std::string modelName = "hippocampus_vision_";      // model name in gazebo
int modelNumber = 0;                        // index
bool check = false;                         // boolean, true if correct model index has been found
int maxModelNumber = 3;                     // maximum number of models in gazebo

// Callback function to subscribe to the gazebo modelStates data
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Find the correct model in the array data
    while(!check) {
        if(!modelName.compare(msg->name[modelNumber])) {
            check = true;
            modelNumber--;
            ROS_INFO("Model %s has been found! gazebo_to_mavros node works correctly!", modelName.c_str());
        }
        modelNumber++;
        if(modelNumber > maxModelNumber) {
            ROS_ERROR("No %s model found! Check gazebo_to_mavros.cpp! Closing the node!", modelName.c_str());
            exit(0);
        }
    }

    // storing the subscribed data in PoseStamped msg to publish them
    msg_out.pose.position.x = msg->pose[modelNumber].position.x;        //position
    msg_out.pose.position.y = msg->pose[modelNumber].position.y;
    msg_out.pose.position.z = msg->pose[modelNumber].position.z;

    msg_out_cov.pose.pose.position.x = msg->pose[modelNumber].position.x;        //position
    msg_out_cov.pose.pose.position.y = msg->pose[modelNumber].position.y;
    msg_out_cov.pose.pose.position.z = msg->pose[modelNumber].position.z;
    msg_out_cov.pose.pose.orientation.x = msg->pose[modelNumber].orientation.x;
    msg_out_cov.pose.pose.orientation.y = msg->pose[modelNumber].orientation.y;
    msg_out_cov.pose.pose.orientation.z = msg->pose[modelNumber].orientation.z;
    msg_out_cov.pose.pose.orientation.w = msg->pose[modelNumber].orientation.w;

/*
    odom.pose.pose.position.x = msg->pose[modelNumber].position.x;
    odom.pose.pose.position.y = msg->pose[modelNumber].position.y;
    odom.pose.pose.position.z = msg->pose[modelNumber].position.z;
    odom.pose.pose.orientation.x = msg->pose[modelNumber].orientation.x;
    odom.pose.pose.orientation.y = msg->pose[modelNumber].orientation.y;
    odom.pose.pose.orientation.z = msg->pose[modelNumber].orientation.z;
    odom.pose.pose.orientation.w = msg->pose[modelNumber].orientation.w;
*/

    /*home.geo.latitude = 28.452386;
    home.geo.longitude = -13.867138;
    home.geo.altitude = 28.5;
    home.position.x = 0.0;
    home.position.y = 0.0;
    home.position.z = 0.0;*/
    //odom.pose.pose.orientation = odom_quat;


    //this can be used for debugging - use loop for multiple vehicles
    /*ROS_INFO("%s positions:\t%8.4f\t%8.4f\t%8.4f",
                msg->name[modelNumber].c_str(),
                msg->pose[modelNumber].position.x,
                msg->pose[modelNumber].position.y,
                msg->pose[modelNumber].position.z);*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_to_mavros");

    ros::NodeHandle n;

    // define rate of publishing and subscribing
    ros::Rate r(20);    // 20 Hz

    // Subscriber
    ros::Subscriber sub = n.subscribe("gazebo/model_states", 1, modelStatesCallback);

    // Publisher
    //ros::Publisher hp_pub = n.advertise<mavros_msgs::HomePosition>("mavros/home_position/home", 2);
    //ros::Publisher pub_mocap = n.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1);
    //ros::Publisher pub_vision = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
    //ros::Publisher pub_vision_cov = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("mavros/vision_pose/pose_cov", 1);
    ros::Publisher pub_loc_pos = n.advertise<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 1);
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("mavros/odometry/odom", 1);
    //ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("mavros/local_position/odom", 1);

    // loop which only ends if this node is killed or ROS is ended
    while(ros::ok()){
        current_time = ros::Time::now();
        //odom.header.stamp = current_time;
        //odom.header.frame_id ="vision_ned";
        //hp_pub.publish(home);
        //pub_mocap.publish(msg_out);		// publishing
        //pub_vision.publish(msg_out);		// publishing
        //pub_vision_cov.publish(msg_out_cov);
        pub_loc_pos.publish(msg_out);		// publishing
        //odom_pub.publish(odom);               // publishing
        //odom_pub2.publish(odom);              // publishing
        ros::spinOnce();			// subscribing
        r.sleep();
    }

    return 0;
}
