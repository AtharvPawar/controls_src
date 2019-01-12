#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include <ostream>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Altitude.h"


using namespace std;

double cur_x=0,cur_y=0, cur_z=0;
double imu_x=0,imu_y=0, imu_z=0, imu_w=0;
bool localization_on=false;

geometry_msgs::PoseStamped loc_pose;
ros::Publisher local_pos_pub;

void Lidardata(mavros_msgs::Altitude height){
  cur_z= height.bottom_clearance;
}


void localization_current(nav_msgs::Odometry cur_local){
  cout << "\033[1;32mLocalization data is coming\033[0m\n"<<endl;
  cur_x=cur_local.pose.pose.position.x;
  cur_y=cur_local.pose.pose.position.y;
  //cur_z=cur_local.pose.pose.position.z;
  cout<<"Current position: "<<cur_x<<","<<cur_y<<","<<cur_z<<endl;

  loc_pose.header.frame_id="map";
  loc_pose.pose.position.x = cur_x;
  loc_pose.pose.position.y = cur_y;
  loc_pose.pose.position.z = cur_z;
  loc_pose.pose.orientation.x = imu_x;
  loc_pose.pose.orientation.y = -imu_y;
  loc_pose.pose.orientation.z = -imu_z;
  loc_pose.pose.orientation.w = imu_w;
  loc_pose.header.stamp=ros::Time::now();

  //---------------------------------
  double roll1, pitch1, yaw1;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (loc_pose.pose.orientation.w * loc_pose.pose.orientation.x + loc_pose.pose.orientation.y * loc_pose.pose.orientation.z);
  double cosr_cosp = +1.0 - 2.0 * (loc_pose.pose.orientation.x * loc_pose.pose.orientation.x + loc_pose.pose.orientation.y * loc_pose.pose.orientation.y);
  roll1 = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (loc_pose.pose.orientation.w * loc_pose.pose.orientation.y - loc_pose.pose.orientation.z * loc_pose.pose.orientation.x);
  if (fabs(sinp) >= 1)
    pitch1 = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch1 = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (loc_pose.pose.orientation.w * loc_pose.pose.orientation.z + loc_pose.pose.orientation.x * loc_pose.pose.orientation.y);
  double cosy_cosp = +1.0 - 2.0 * (loc_pose.pose.orientation.y * loc_pose.pose.orientation.y + loc_pose.pose.orientation.z * loc_pose.pose.orientation.z);  
  yaw1 = atan2(siny_cosp, cosy_cosp);

  cout << "Roll: " << roll1 << "\nPitch: " << pitch1 << "\nYaw: " << yaw1 << endl;
  //-----------------------------------

  local_pos_pub.publish(loc_pose);
  cout << "\033[1;33mPublished\033[0m\n"<<endl;
}

void imudata(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  imu_x=msg->orientation.x;
  imu_y=msg->orientation.y;
  imu_z=msg->orientation.z;
  imu_w=msg->orientation.w;
  cout<<"Current orientation: "<<imu_x<<","<<imu_y<<","<<imu_z<<","<<imu_w<<endl;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_vpe");

  ros::NodeHandle n;
  ros::Rate rate(10);
  ros::Duration(1.0).sleep();

  //ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 2, imudata);
  ros::Subscriber localization_sub = n.subscribe("/odom", 2, localization_current);
  ros::Subscriber height_sub = n.subscribe("/mavros/altitude", 2, Lidardata);

  local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 2);

  while(ros::ok()){

  // loc_pose.header.frame_id="base_link";
  // loc_pose.pose.position.x = 0;
  // loc_pose.pose.position.y = 0;
  // loc_pose.pose.position.z = 0;
  // loc_pose.pose.orientation.x = imu_x;
  // loc_pose.pose.orientation.y = imu_y;
  // loc_pose.pose.orientation.z = imu_z;
  // loc_pose.pose.orientation.w = imu_w;
  // loc_pose.header.stamp=ros::Time::now();

  // local_pos_pub.publish(loc_pose);
  // cout<<"Published"<<endl;
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}