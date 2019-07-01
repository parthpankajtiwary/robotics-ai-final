#include <headController/headController.h>

HeadController::HeadController(ros::NodeHandle &nh) {
  current_pitch_ = 0.0f;
  current_yaw_ = 0.0f;
  new_pitch_ = 0.0f;
  new_yaw_ = 0.0f;
  pitch_subscriber_ = nh.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state", 1, &HeadController::pitchCallback, this);
  yaw_subscriber_ = nh.subscribe<dynamixel_msgs::JointState>("pan_controller/state", 1, &HeadController::yawCallback, this);
  joy_subscriber_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &HeadController::joyCallback, this);

  pitch_publisher_ = nh.advertise<std_msgs::Float64>("tilt_controller/command", 1);
  yaw_publisher_ = nh.advertise<std_msgs::Float64>("pan_controller/command", 1);

}

void HeadController::joyCallback(const sensor_msgs::JoyConstPtr &data) {
  std::vector<float> axes = data->axes;

  if (axes.at(7) < -0.9) {
    down_ = true;
    up_ = false;
  } else if (axes.at(7) > 0.9) {
    down_ = false;
    up_ = true;
  } else {
    down_ = false;
    up_ = false;
  }

  if (axes.at(6) < -0.9) {
    left_ = false;
    right_ = true;
  } else if (axes.at(6) > 0.9) {
    left_ = true;
    right_ = false;
  } else {
    left_ = false;
    right_ = false;
  }
}

void HeadController::pitchCallback(const dynamixel_msgs::JointStateConstPtr &data) {
  current_pitch_ = data->current_pos;

  std_msgs::Float64 new_pitch;
  if (up_) {
    new_pitch.data = current_pitch_ + step_;
    pitch_publisher_.publish(new_pitch);
  } else if (down_) {
    new_pitch.data = current_pitch_ - step_;
    pitch_publisher_.publish(new_pitch);
  }  
}

void HeadController::yawCallback(const dynamixel_msgs::JointStateConstPtr &data) {
  current_yaw_ = data->current_pos;

  std_msgs::Float64 new_yaw;
  if (left_) {
    new_yaw.data = current_yaw_ + step_;
    yaw_publisher_.publish(new_yaw);
  } else if (right_) {
    new_yaw.data = current_yaw_ - step_;
    yaw_publisher_.publish(new_yaw);
  }  
}
