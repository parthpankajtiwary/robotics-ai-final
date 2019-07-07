#ifndef __HEADCONTROLLER_H
#define __HEADCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

class HeadController {

public: 
 HeadController(ros::NodeHandle &nh);

private: 
 ros::Subscriber pitch_subscriber_;
 ros::Subscriber yaw_subscriber_;
 ros::Subscriber joy_subscriber_;

 ros::Publisher pitch_publisher_;
 ros::Publisher yaw_publisher_;
 
 void pitchCallback(const dynamixel_msgs::JointStateConstPtr &data);
 void yawCallback(const dynamixel_msgs::JointStateConstPtr &data);
 void joyCallback(const sensor_msgs::JoyConstPtr &data);

 float current_pitch_;
 float current_yaw_;

 float step_{0.1};
 float new_pitch_;
 float new_yaw_;

 bool up_;
 bool down_;
 bool left_;
 bool right_;
};

#endif