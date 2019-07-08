#include "alice_headcontroller.h"

bool AliceHeadcontroller::MoveHeadCallback(alice_msgs::MoveHeadRequest &request,
                                      alice_msgs::MoveHeadResponse &responce) {
  float pitch, yaw;
  pitch = request.pitch;
  yaw = request.yaw;

  stop_checking_vel = true;
  time = ros::Time::now();
  MoveHead(pitch, yaw);
  return true;
}

void AliceHeadcontroller::MoveHead(float &pitch, float &yaw, bool wait) {
  std_msgs::Float64 pitch_msg, yaw_msg;

  pitch_msg.data = pitch;
  yaw_msg.data = yaw;

  pitch_command_.publish(pitch_msg);
  yaw_command_.publish(yaw_msg);

  if (wait) {
    WaitForMovementToBeDone(pitch, yaw);
  }
}

void AliceHeadcontroller::WaitForMovementToBeDone(float &pitch, float &yaw) {

  bool pitch_done = false;
  bool yaw_done = false;
  bool done = false;
  float current_pitch, current_yaw;

  while (!done) {
    current_pitch = GetCurrentPosition(pitch_state_topic_);
    current_yaw = GetCurrentPosition(yaw_state_topic_);

    // If error is smaller than 2 degrees, assume it has arrived
    if (std::fabs(current_pitch - pitch) < ToRadian(2.0)) {
      pitch_done = true;
    }
    if (std::fabs(current_yaw - yaw) < ToRadian(2.0)) {
      yaw_done = true;
    }

    done = pitch_done && yaw_done;
  }
}
