#include <speedController/speedController.h>

SpeedController::SpeedController(ros::NodeHandle &nh) :
  yaw_step_(0.05f), x_step_(0.05f) {
  
  current_x_speed_ = 0.0f;
  current_yaw_speed_ = 0.0f;

  x_joystick_ = 0.0f;
  yaw_joystick_ = 0.0f;
  max_x_speed_ = 0.4f;
  max_yaw_speed_ = 0.5f;

  ticker_ = nh.createTimer(ros::Duration(0.1), &SpeedController::updateVelocity, this, false, false);
  ticker_.start();
  velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_subscriber_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &SpeedController::joyCallback, this);
}

void SpeedController::updateVelocity(const ros::TimerEvent &t) {

  if (std::fabs(current_x_speed_) < max_x_speed_) {
    current_x_speed_ += x_joystick_ * x_step_;
    
    if (current_x_speed_ > max_x_speed_) {
        current_x_speed_ = max_x_speed_;
    } else if (current_x_speed_ < -max_x_speed_) {
        current_x_speed_ = -max_x_speed_;
    }
  }

  if (std::fabs(current_yaw_speed_) < max_yaw_speed_) {
    current_yaw_speed_ += yaw_joystick_ * yaw_step_;
    
    if (current_yaw_speed_ > max_yaw_speed_) {
        current_yaw_speed_ = max_yaw_speed_;
    } else if (current_yaw_speed_ < -max_yaw_speed_) {
        current_yaw_speed_ = -max_yaw_speed_;
    }
  } 

  if (stop_) {
    current_x_speed_ = 0.0f;
    current_yaw_speed_ = 0.0f;
  }

  current_x_speed_ *= 0.95;
  current_yaw_speed_ *= 0.95;

  if (std::fabs(current_x_speed_) < 0.001f) {
    current_x_speed_ = 0.0f;
  }

  if (std::fabs(current_yaw_speed_) < 0.001f) {
    current_yaw_speed_ = 0.0f;
  }

  geometry_msgs::Twist velocity_msg;
  velocity_msg.linear.x = current_x_speed_;
  velocity_msg.angular.z = current_yaw_speed_;

  velocity_publisher_.publish(velocity_msg);
}

void SpeedController::joyCallback(const sensor_msgs::JoyConstPtr &data) {
  std::vector<float> axes = data->axes;
  std::vector<int> buttons = data->buttons;

  x_joystick_ = axes.at(4);
  yaw_joystick_ = axes.at(0);

  stop_ = data->buttons.at(4) == 1 || data->buttons.at(5) == 1;
}