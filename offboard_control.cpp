/** Hi
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/String.h"
#include <string>

using namespace std;

string command = "";
bool isNewRequest = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void commands(const std_msgs::String::ConstPtr& msg){
    command = msg->data.c_str();
    cout << command << endl;
    isNewRequest = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber comm_sub = nh.subscribe("/commands", 2, commands);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if (command == "a") {
            arm_cmd.request.value = true;
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }
        }
        if (command == "d") {
            arm_cmd.request.value = false;
            if( current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }
        }
        if (command == "o") {
            for(int i = 100; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if ( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }

        }
        if (command == "oo") {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1;
            for(int i = 20; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            isNewRequest = false;
        }
        if (command == "ox") {
            pose.pose.position.x = 0;
            pose.pose.position.y = -0.5;
            pose.pose.position.z = 1;
            for(int i = 20; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            isNewRequest = false;
         }
        else if (command == "l") {
            offb_set_mode.request.custom_mode = "ALTCTL";
            if( current_state.mode != "ALTCTL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if ( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ) {
                    ROS_INFO("Altitude control enabled");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }   
        }
        if (command == "p") {
            offb_set_mode.request.custom_mode = "POSCTL";
            if( current_state.mode != "POSCTL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if ( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ) {
                    ROS_INFO("Position control enabled");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }   
        }
        if (command == "m") {
            offb_set_mode.request.custom_mode = "MANUAL";
            if( current_state.mode != "MANUAL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0) || isNewRequest ) ) {
                if ( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ) {
                    ROS_INFO("Manual mode enabled");
                }
                last_request = ros::Time::now();
                isNewRequest = false;
            }   
        }
        if (command == "x");  //stop trying to do anything

        //local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}