#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "keyboard_commands");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("/commands", 1);
  string input;
  ros::Rate loop_rate(10);

  std_msgs::String msg;
  std::stringstream ss;

  while (ros::ok()) {
    std::stringstream ss;
    cin >> input;
    ss << input;
    msg.data = ss.str();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}