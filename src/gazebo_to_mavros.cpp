/*
*   gazebo_to_mavros node to get information such as the position from gazebo transported to the PX4 Firmware
*   Nils Rottmann
*   Nils.Rottmann@tuhh.de
*/


#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>                // including the required messages
#include <geometry_msgs/PoseStamped.h>

/*
This topic subscribes to the gazebo ros topic /gazebo/model_states to get the state of the different models in the gazebo simulation
and publishes it to the /mavros/mocap/pose topic of the mavros package.
*/

//using global coordinates to store the subscribed data
geometry_msgs::PoseStamped msg_out;

// Callback function to subscribe to the gazebo modelStates data
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int i = 1;    // this is the model number of the hippocampus in the gazebo simulator
    msg_out.pose.position.x = msg->pose[i].position.x;      // storing the subscribed data in PoseStamped msg to publish them
    msg_out.pose.position.y = msg->pose[i].position.y;
    msg_out.pose.position.z = msg->pose[i].position.z;

    /*// this can be used for debugging
    ROS_INFO("%s positions:\t%8.4f\t%8.4f\t%8.4f",
                msg->name[i].c_str(),
                msg->pose[i].position.x,
                msg->pose[i].position.y,
                msg->pose[i].position.z);*/
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
    ros::Publisher pub_mocap = n.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1);

    // loop which only ends if this node is killed or ROS is ended
    while(ros::ok()){
        pub_mocap.publish(msg_out);		// publishing
        ros::spinOnce();			// subscribing
        r.sleep();
    }

    return 0;
}
