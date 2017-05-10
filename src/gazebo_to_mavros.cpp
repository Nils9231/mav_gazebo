/*
*   gazebo_to_mavros node to get information such as the position from gazebo transported to the PX4 Firmware
*   Nils Rottmann
*   Nils.Rottmann@tuhh.de
*/


#include <string>
#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>                // including the required messages
#include <geometry_msgs/PoseStamped.h>

/*
This topic subscribes to the gazebo ros topic /gazebo/model_states to get the state of the different models in the gazebo simulation
and publishes it to the /mavros/mocap/pose topic of the mavros package.
*/

//using global coordinates to store the subscribed data
geometry_msgs::PoseStamped msg_out;

// Global variables to find the model number of the array
std::string modelName = "hippocampus";      // model name in gazebo
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
    msg_out.pose.position.x = msg->pose[modelNumber].position.x;
    msg_out.pose.position.y = msg->pose[modelNumber].position.y;
    msg_out.pose.position.z = msg->pose[modelNumber].position.z;

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
