#include "alice_controller.h"

void AliceController::CallbackOdom(const nav_msgs::OdometryConstPtr &odom) {

  if (get_first_position_) {
    start_odom_ = *odom;
    get_first_position_ = false;
  }

  if (control_state_ == ControlState::MOVE) {
    Move(odom);
  } else if (control_state_ == ControlState::TURN) {
    Turn(odom);
  } else {
    control_state_ = ControlState::STOP;
  }
}

void AliceController::StartOdomSubscriber() {
  odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&AliceController::CallbackOdom, this, _1));
}

void AliceController::StopOdomSubscriber() {
  odom_subscriber_.shutdown();
}

void AliceController::Move(const nav_msgs::OdometryConstPtr &odom) {
  float distance = std::sqrt(std::pow(start_odom_.pose.pose.position.x - odom->pose.pose.position.x,2) +
                             std::pow(start_odom_.pose.pose.position.y - odom->pose.pose.position.y, 2));


  if (std::fabs(distance - distance_) <= distance_threshold_) {
    SendTwist(0, 0);
    control_state_ = ControlState::STOP;
    as_.setSucceeded();
    return;
  }

  SendTwist(speed_, 0);
}

void AliceController::Turn(const nav_msgs::OdometryConstPtr &odom) {
  float current_yaw = tf::getYaw(odom->pose.pose.orientation);
  float angle_step = 0.0f;

  if (!first_angle_) {
    angle_step = std::fabs(current_yaw - prev_angle_);
  } else {
    first_angle_ = false;
  }
  rotation_distance_ += angle_step;

  if (rotation_distance_ >= std::fabs(angle_ * M_PI/180.0)) {
    SendTwist(0, 0);
    control_state_ = ControlState::STOP;
    as_.setSucceeded();
    return;
  }

  prev_angle_ = current_yaw;
  SendTwist(0, rotation_speed_);
}

void AliceController::SendTwist(float x, float yaw) {
  geometry_msgs::Twist twist;
  twist.linear.x = x;
  twist.angular.z = yaw;

  cmd_vel_publisher_.publish(twist);
}
