/*
* ros_simple_app to demonstrate the possibilities to control the PX4 simulation in gazebo via ROS
* Nils Rottmann
* Nils.Rottmann@tuhh.de
 */

#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>    // actuator control to send signals to px4 via MAVROS
#include <geometry_msgs/PoseStamped.h>      // to receive position messages

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pos_msg){
    // publish the current position into the terminal
    ROS_INFO("Positions:\t%8.4f\t%8.4f\t%8.4f",
                pos_msg->pose.position.x,
                pos_msg->pose.position.y,
                pos_msg->pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_simple_app");
    ros::NodeHandle nh;

    // define rate of publishing and subscribing
    ros::Rate rate(20);    // 20 Hz
    
    // define the subscriber to the mavros/mocap/pose topic
    ros::Subscriber pos_sub = nh.subscribe("mavros/mocap/pose", 1, position_callback);

    // define the publisher to the actuator control topic
    ros::Publisher local_act_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 1);

    // Define the published actuator control messages
    mavros_msgs::ActuatorControl control;
    control.controls[0] = 0.0;
    control.controls[1] = 0.0;
    control.controls[2] = 1.0;
    control.controls[3] = 1.0;
    control.controls[4] = 0.0;
    control.controls[5] = 0.0;
    control.controls[6] = 0.0;
    control.controls[7] = 0.0;

    // loop which only ends if this node is killed or ROS is ended
    while(ros::ok()){
	local_act_pub.publish(control);	// publishing
        ros::spinOnce();		// subscribing
        rate.sleep();
    }

 

    return 0;
}
