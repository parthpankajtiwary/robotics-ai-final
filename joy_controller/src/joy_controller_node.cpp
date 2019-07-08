#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <speedController/speedController.h>
#include <headController/headController.h>

ros::Publisher velocity_publisher;


void joyCallback(const sensor_msgs::JoyConstPtr &data) {
  std::vector<float> axes = data->axes;
  std::vector<int> buttons = data->buttons;

  float forward = axes.at(4);

  geometry_msgs::Twist velocity_message;
  velocity_message.linear.x = forward * 0.3;


  if (buttons.at(4) == 1 || buttons.at(5) == 1) {
    velocity_message.linear.x = 0.0f;
    velocity_message.angular.z = 0.0f;
  }

  velocity_publisher.publish(velocity_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_controller");
  ros::NodeHandle nh;
  SpeedController speedController(nh);
  HeadController headController(nh);
  ros::spin();
}