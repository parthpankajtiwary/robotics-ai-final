#ifndef __SPEEDCONTROLLER_H_
#define __SPEEDCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>

class SpeedController {

public:
 SpeedController(ros::NodeHandle &nh);

private: 
 void joyCallback(const sensor_msgs::JoyConstPtr &data);
 void updateVelocity(const ros::TimerEvent &);

 ros::Publisher velocity_publisher_;
 ros::Subscriber joy_subscriber_;

 float current_x_speed_;
 float current_yaw_speed_;

 float max_x_speed_;
 float max_yaw_speed_;

 const float x_step_;
 const float yaw_step_;

 float x_joystick_;
 float yaw_joystick_;
 bool stop_;

 ros::Timer ticker_;
};

#endif