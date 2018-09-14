/**
 * offb_node, to set OFFBOARD mode and arm the PX4 simulation
 * Nils Rottmann
 * Nils.Rottmann@tuhh.de
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // initialize the offb_node node
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // states subscriber to check if arming and mode change has been successful
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // Clients for arming and mode changing
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;                 // instantiate an autogenerated service class
    offb_set_mode.request.custom_mode = "OFFBOARD";     // assign values into its request member

    mavros_msgs::CommandBool arm_cmd;                   // same as above
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if(ros::ok()){                                      // check if the mode is OFFBOARD and the system is armed
        if( current_state.mode != "OFFBOARD") {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        }

        ros::Duration(5).sleep();                       // sleep for 5 second to not overcharge the system

        if( !current_state.armed){
            if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
        }

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
