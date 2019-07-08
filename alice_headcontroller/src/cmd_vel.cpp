#include "alice_headcontroller.h"

void AliceHeadcontroller::TwistCallback(const geometry_msgs::TwistConstPtr &twist) {
  
  if (!stop_checking_vel) {
      // need to move the tilt and yaw based on commands given here
      // most likely want to filter it to it doesn't move fast and twitchy..

      // from current pos move to new pos, based on command
      float x = twist->linear.x;
      float z = twist->angular.z;

      if (x > 0) {
        float new_pitch = min_pitch_ + x;

        pitch_position_ = pitch_position_ * (1.0f - filter_) + new_pitch * filter_;
      }

      float new_yaw = max_yaw_ / 0.5 * z;
      //std::cout << new_yaw << ", " << z << "\n";

      yaw_position_ = yaw_position_ * (1.0f - filter_yaw_) + new_yaw * filter_yaw_;

      MoveHead(pitch_position_, yaw_position_, false);
  }
  
  if (stop_checking_vel && ros::Time::now() - time > ros::Duration(10)) {
    stop_checking_vel = false;
  }
}
